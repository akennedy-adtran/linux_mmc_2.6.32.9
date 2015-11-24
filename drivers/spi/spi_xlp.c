/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BROADCOM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL BROADCOM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * #BRCM_2# */


#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/io.h>

#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp.h>

#undef XLP_SPI_DEBUG

#define CONFIG_SPI_REFCLK		133333334
#define SPI_CHAN_OFFSET			0x10
#define CMD_RDID			0x9f
#define CMD_RDSR			0x05
#define CMD_WREN			0x06
#define CMD_WRDI			0x04
#define CMD_BE_4K			0x20
#define CMD_BE_32K			0x52
#define CMD_CHIP_ERASE			0xc7
#define CMD_SE				0xd8
#define CMD_GET_FEATURE			0x0f
#define CMD_SET_FEATURE			0x1f
#define CMD_RESET			0xff
#define CMD_PR				0x13
#define CMD_PL				0x02
#define CMD_PROGEXE			0x10

#define DEFAULT_CS_FDIV			0x10
#define XLP_SPI_MAX_XFER_SIZE		0x2000
#define XLP_SPI_FIFO_SIZE		8
#define NOR_SPI_CMD_SIZE		5
#define XLP_SPI_MAX_CS			4

#define SPI_CFG_CPHA(x)                 ((x) << 0 )     
#define SPI_CFG_CPOL(x)                 ((x) << 1 )
#define SPI_CFG_CSPOL(x)                ((x) << 2 )
#define SPI_CFG_TXMISO(x)               ((x) << 3 )
#define SPI_CFG_TXMOSI(x)               ((x) << 4 )
#define SPI_CFG_RXMISO(x)               ((x) << 5 )
#define SPI_CFG_CSTODATA(x)     	((x) << 8 )
#define SPI_CFG_LSBFE(x)                ((x) << 10)
#define SPI_CFG_RXCAP(x)                ((x) << 11)

struct spi_xlp {
	struct spi_bitbang bitbang;
	struct completion done;
	uint8_t 	cs;
	uint8_t 	cs_active;
	void __iomem	*regs;
	uint32_t	irq;
	uint32_t	speed_hz;
	unsigned char	*rx_buf;
	unsigned char	*tx_buf;
	int32_t 	rcounter;
};

static __inline__ int32_t spi_reg_read(int node, int cs, int regidx)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_CTRL);
	regidx +=  cs * SPI_CHAN_OFFSET;
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static __inline__ void spi_reg_write(int node, int cs, int regidx, int32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_CTRL);
	regidx +=  cs * SPI_CHAN_OFFSET;
        nlm_hal_write_32bit_reg(mmio, regidx, val);
}

#ifdef XLP_SPI_DEBUG
static void spi_dump_reg()
{
        int i, j = 0;
	for(i = 0; i < 8; i++) {
		printk("0x%0x = 0x%8x\n", i, spi_reg_read( 0, j, i));
	}
        for(j = 0; j < XLP_SPI_MAX_CS; j++)
        {
                printk("dump spi_%d register\n", j);
                for(i = 0x40; i < 0x47; i++) {
                        printk("0x%0x = 0x%8x\n", i, spi_reg_read( 0, j, i));
                }
        }
        printk("0x%0x = 0x%8x\n", 0x80, spi_reg_read( 0, 0, 0x80));
}
#endif

static void xlp_spi_init(struct spi_xlp *pspi)
{
	uint32_t i, val;

	val = spi_reg_read(0, 0, XLP_SPI_SYSCTRL);
	val |= XLP_SPI_SYS_PMEN;
        spi_reg_write(0, 0, XLP_SPI_SYSCTRL, (XLP_SPI_SYS_RESET << pspi->cs));

	pspi->cs_active = 0;
	for(i = 0; i < XLP_SPI_MAX_CS; i++)
	{
		val = spi_reg_read(0, i, XLP_SPI_CONFIG);
		val |= XLP_SPI_TXMOSI_EN |XLP_SPI_RXMISO_EN;
		spi_reg_write(0, i, XLP_SPI_CONFIG, val);
	}
}

static void spi_xlp_chipselect(struct spi_device *spi, int is_on)
{
	struct spi_xlp *pspi = spi_master_get_devdata(spi->master);

        if(pspi)
                pspi->cs =  spi->chip_select;

	if (is_on == BITBANG_CS_INACTIVE) {
		pspi->cs_active = 0;
	} else if (is_on == BITBANG_CS_ACTIVE) {
		pspi->cs_active = 1;
	}
}

static int spi_xlp_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{
	uint8_t bits_per_word;
	uint32_t hz;
	struct spi_xlp *pspi = spi_master_get_devdata(spi->master);

	bits_per_word = (t) ? t->bits_per_word : spi->bits_per_word;
	hz = (t) ? t->speed_hz : spi->max_speed_hz;

	if (hz && pspi->speed_hz > hz) {
		printk("%s, unsupported clk rate %uHz\n", __func__, hz);
		return -EINVAL;
	}

	return 0;
}

static int spi_xlp_setup(struct spi_device *spi)
{
	struct spi_xlp *pspi;
	int32_t val;
	uint32_t reg_val;
	uint64_t refclk;
	uint32_t max_hz = CONFIG_SPI_REFCLK/8;


	pspi = spi_master_get_devdata(spi->master);
	pspi->cs = spi->chip_select;

	spi_reg_write(0, 0, XLP_SPI_SYSCTRL, XLP_SPI_SYS_PMEN);
	refclk = nlm_hal_get_ref_clk_freq();
	
	if(max_hz <= refclk/(64*1024) || refclk/4 <= max_hz )	{
		printk("Unsupported SPI frequency %d\n",max_hz);
		return -1;
	}
	//pspi->speed_hz = refclk/8;
	spi_reg_write(0, spi->chip_select, XLP_SPI_FDIV, do_div(refclk,max_hz));
	spi_reg_write(0, spi->chip_select, XLP_SPI_FIFO_THRESH, 0x8 || (0x8 << 4));
	reg_val = (
			SPI_CFG_CPHA(   (spi->mode & XLP_SPI_CPHA)     ? 1 : 0)
			| SPI_CFG_CPOL(   (spi->mode & XLP_SPI_CPOL)     ? 1 : 0)
			| SPI_CFG_CSPOL(  (spi->mode & XLP_SPI_CS_POL_HI)  ? 0 : 1)  //Need reverse on PRM
			| SPI_CFG_TXMISO( (spi->mode & 0x10)    ? 1 : 0)
			| SPI_CFG_TXMOSI( 1 ) 
			| SPI_CFG_RXMISO( 1 ) 
			| SPI_CFG_LSBFE(  (spi->mode & 0x08)? 1 : 0)
			| SPI_CFG_CSTODATA(0) );
	
	spi_reg_write(0, spi->chip_select, XLP_SPI_CONFIG, reg_val);
	#ifdef XLP_SPI_DEBUG
		spi_dump_reg();
	#endif
	val = spi_xlp_setup_transfer(spi, NULL);
	if (val < 0)
		return val;
	return 0;

}

static void spi_xlp_fill_txfifo(struct spi_xlp *pspi, uint32_t* len, unsigned char** data)
{
        uint32_t txfifo_cnt;
        uint32_t tx_data;
        txfifo_cnt = spi_reg_read(0, pspi->cs, XLP_SPI_FIFO_WCNT);
	txfifo_cnt >>= XLP_SPI_TXFIFO_WCNT_POS;

        while ((*len) && (txfifo_cnt < XLP_SPI_FIFO_SIZE))
        {
                if (*len <= 1) {
                        tx_data = (*data)[0];
                        *len = 0;
                } else if (*len <= 2) {
                        tx_data = (((*data)[0] << 8) |
                                   ((*data)[1] << 0));
                        *len = 0;
                } else if (*len <= 3) {
                        tx_data = (((*data)[0] << 16) |
                                    ((*data)[1] << 8) |
                                    ((*data)[2] << 0));
                        *len = 0;
                } else {
                        tx_data = (((*data)[0] << 24) |
                                   ((*data)[1] << 16) |
                                   ((*data)[2] << 8)  |
                                   ((*data)[3] << 0));
                        *len = *len - 4;
                }

		spi_reg_write(0, pspi->cs, XLP_SPI_TXDATA_FIFO, tx_data);
                (*data) += 4;
                txfifo_cnt++;
        }
}


static int spi_xlp_xfer_block(struct spi_device *spi, struct spi_transfer *t, uint32_t xfer_len, u8 spi_cmd_cont)
{
	uint32_t val;
	uint32_t rx_data, tx_len, rx_len, rxfifo_cnt, sent_bytes;
	struct spi_xlp *pspi = spi_master_get_devdata(spi->master);
	unsigned char* rx_buf = NULL;
	unsigned char* tx_buf = NULL;

	tx_buf = (unsigned char*)t->tx_buf;
	rx_buf = (unsigned char*)t->rx_buf;

	if (!tx_buf && !rx_buf)
                return -1;

	tx_len = (tx_buf == NULL) ? 0 : xfer_len;
	rx_len = (rx_buf == NULL) ? 0 : xfer_len;
	sent_bytes = 0;

	val = XLP_SPI_CMD_IDLE;
	if (tx_len) {
		spi_xlp_fill_txfifo(pspi, &tx_len, &tx_buf);
		sent_bytes = xfer_len - tx_len;
        }

	if(spi_cmd_cont)
		val |= XLP_SPI_CMD_CONT;

	if(t->tx_buf)
		val |= XLP_SPI_CMD_TX;
	if(t->rx_buf)
		val |= XLP_SPI_CMD_RX;
        if(xfer_len)
		val |= ((xfer_len * 8 - 1) << XLP_SPI_XFR_BITCNT_POS);

	spi_reg_write(0, pspi->cs, XLP_SPI_CMD, val);

        while ((tx_len) || (rx_len)) {

                if (rx_len) {

			rxfifo_cnt = spi_reg_read(0, pspi->cs, XLP_SPI_FIFO_WCNT);
			rxfifo_cnt = (0xF & rxfifo_cnt);

                        while (rxfifo_cnt) {

				rx_data = spi_reg_read(0, pspi->cs, XLP_SPI_RXDATA_FIFO);
				rxfifo_cnt--;

                                if (rx_len <= 1) {
                                        rx_buf[0] = (uint8_t) (rx_data & 0xff);
                                        rx_len = 0;
                                } else if (rx_len <= 2) {
                                        rx_buf[0] = (uint8_t) ((rx_data >> 8) & 0xff);
                                        rx_buf[1] = (uint8_t) ((rx_data >> 0) & 0xff);
                                        rx_len = 0;
                                } else if (rx_len <= 3) {
                                        rx_buf[0] = (uint8_t) ((rx_data >> 16) & 0xff);
                                        rx_buf[1] = (uint8_t) ((rx_data >> 8) & 0xff);
                                        rx_buf[2] = (uint8_t) ((rx_data >> 0) & 0xff);
                                        rx_len = 0;
                                } else {
                                        rx_buf[0] = (uint8_t) ((rx_data >> 24) & 0xff);
                                        rx_buf[1] = (uint8_t) ((rx_data >> 16) & 0xff);
                                        rx_buf[2] = (uint8_t) ((rx_data >> 8) & 0xff);
                                        rx_buf[3] = (uint8_t) ((rx_data >> 0) & 0xff);
                                        rx_len -= 4;
                                }
                                rx_buf += 4;
                        }
			sent_bytes = xfer_len - rx_len;
                }
                if (tx_len) {
			spi_xlp_fill_txfifo(pspi, &tx_len, &tx_buf);
			sent_bytes = xfer_len - tx_len;	
                }
        }

	do {
		val = spi_reg_read(0, pspi->cs, XLP_SPI_STATUS);
		if(val & XLP_SPI_TX_OV_TH)
		{
			printk("[%s] tx over threshold, stop sendinging\n",__func__);
		}
        } while((val & XLP_SPI_XFR_DONE) == 0);

        return sent_bytes;
}

static int spi_xlp_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
        int ret;
	uint32_t len;

	ret = 0;
	len = t->len;
	while (len > XLP_SPI_MAX_XFER_SIZE) {
		ret += spi_xlp_xfer_block(spi, t, XLP_SPI_MAX_XFER_SIZE, 1);
		len = len - XLP_SPI_MAX_XFER_SIZE;
		if (t->tx_buf)
			t->tx_buf = t->tx_buf + XLP_SPI_MAX_XFER_SIZE;
		if (t->rx_buf)
			t->rx_buf = t->rx_buf + XLP_SPI_MAX_XFER_SIZE;
	}
        ret += spi_xlp_xfer_block(spi, t, len, t->spi_cont_cmd);

        return ret;
}

static int __init spi_xlp_probe(struct platform_device *dev)
{
	int ret = 0;
	struct spi_master *master;
	struct spi_xlp *pspi;
	struct resource *r;

	master = spi_alloc_master(&dev->dev, sizeof(struct spi_xlp));

	if (master == NULL) {
		return -ENOMEM;
	}

	dev_set_drvdata(&dev->dev, master);

	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENODEV;
		goto put_master;
	}

	pspi = spi_master_get_devdata(master);
	pspi->bitbang.master 		= spi_master_get(master);
	pspi->bitbang.chipselect	= spi_xlp_chipselect;
	pspi->bitbang.setup_transfer	= spi_xlp_setup_transfer;
	pspi->bitbang.txrx_bufs		= spi_xlp_txrx_bufs;
	pspi->bitbang.master->setup	= spi_xlp_setup;
	init_completion(&pspi->done);

	if (!request_mem_region(r->start,
			r->end - r->start + 1, "spi-xlp")) {
		ret = -ENXIO;
		goto put_master;
	}

	pspi->regs = ioremap(r->start, r->end - r->start + 1);
	if (pspi->regs == NULL) {
		ret = -ENOMEM;
		goto put_master;
	}

	master->bus_num = 0;
	master->num_chipselect = XLP_SPI_MAX_CS;

	xlp_spi_init(pspi);
	ret = spi_bitbang_start(&pspi->bitbang);
	if (ret != 0) {
		dev_err(&dev->dev, "spi_bitbang_start FAILED\n");
		goto unmap_io;
	}

	return ret;

unmap_io:
	iounmap(pspi->regs);
put_master:
	spi_master_put(master);
	return ret;
}

static int __devexit spi_xlp_remove(struct platform_device *dev)
{
	struct spi_xlp *pspi;
	struct spi_master *master;

	master = platform_get_drvdata(dev);
	pspi = spi_master_get_devdata(master);

	spi_bitbang_stop(&pspi->bitbang);
	iounmap(pspi->regs);
	platform_set_drvdata(dev, 0);
	spi_master_put(pspi->bitbang.master);

	return 0;
}

static struct platform_driver spi_xlp_driver = {
	.probe	= spi_xlp_probe,
	.remove	= __devexit_p(spi_xlp_remove),
	.driver = {
		.name = "spi-xlp",
		.owner = THIS_MODULE,
	},
};

static int __init spi_xlp_init(void)
{
	return platform_driver_register(&spi_xlp_driver);
}
module_init(spi_xlp_init);

static void __exit spi_xlp_exit(void)
{
	platform_driver_unregister(&spi_xlp_driver);
}
module_exit(spi_xlp_exit);

MODULE_AUTHOR("Netlogic Microsystem Inc.");
MODULE_DESCRIPTION("Netlogic XLP SPI master controller driver");
MODULE_LICENSE("GPL");
