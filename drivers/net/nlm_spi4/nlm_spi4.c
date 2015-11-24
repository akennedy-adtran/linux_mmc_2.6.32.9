/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
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


#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cache.h>

#include "os_layer.h"
#include "nlm_spi4.h"
#include "nlm_spi4_config.h"
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/msgring.h>
#include "nlm_vits_driver.h"

#define CLEAR_INT 0xf
#define CLEAR_TX_STATUS 0x0f
#define CLEAR_RX_STATUS 0x1f
#define MAX_BUCKET 64
#define NLM_SPI4_MAX_THREADS 32
#define NLM_SPI4_DEBUG 0
#define MAX_NUM_MSGRNG_STN_CC   128

unsigned long g_dip4_error[NLM_SPI4_MAX_THREADS];

extern int xlr_loader_support;
extern int xlr_loader_sharedcore;
//extern void rmik_config_pde(int type, int instance, nlm_reg_t *mmio);

spi4_driver_data*	spi4_data[TOTAL_SPI4];

static void spi4_msgring_handler(	int 	bucket, 
					int 	size, 
					int 	code, 
					int 	stid, 
					struct msgrng_msg *msg, 
					void 	*data);

extern void rx_indication(int port, 	char* addr, 	int len);
extern void tx_complete(int port, 	char* addr);

#if NLM_SPI4_DEBUG
void spi4_init_read_counter(unsigned int	*mmio)
{
	mmio[0x236] = 0; // read count0 of debugcount0
	mmio[0x237] = 0; // read count1 of debugcount0

	mmio[0x23a] = 0; // read count0 of debugcount1
	mmio[0x23b] = 0; // read count1 of debugcount1

	return;
}

void spi4_write_select(unsigned int  *mmio,int write_select,	int bit_no)
{
	long value;
	value = 1<<bit_no;
	printk("write_select=%d, bit no=%d,value=0x%lx\n", 
		write_select, bit_no,value);
	switch(write_select){
	case 0:
		mmio[0x234] = value;
		break;
	case 1:
		mmio[0x235] = value;
		break;
	case 2:
		mmio[0x238] = value;
		break;
	case 3:
		mmio[0x239] = value;
		break;
	default:
		printk("wrong write select\n");
	}// end of switch
	return;
}

void spi4_print_debug_value(unsigned int  *mmio)
{

	printk(" Read counter0 = %d\n", mmio[0x236]);
	printk(" Read counter1 = %d\n", mmio[0x237]);
	printk(" Read counter3 = %d\n", mmio[0x23a]);
	printk(" Read counter4 = %d\n", mmio[0x23b]);

	return;
}

#endif
/*******************************************************************************
*
* Function name	:	spi4_configure_pde_spray_mode
* Input		:	spi4 slot
* Description	:	This function will program the PDE for the given 
*	spi4 slot. Based on the cpu mask, it will check the enabled cpu and 
*	then calculates the bucket mapp, with this bucket map it will program
*	all the 4 classes of PDE .
*	
* RETURNS: void
*******************************************************************************/
static int spi4_configure_pde_spray_mode(uint32 slot, struct port_cfg *pcfg)
{

	int i, cpu=0, bkt=0;
	__u64 bucket_map = 0;
	bucket_t *bucket;
	struct stn_cc *credit;
	spi4_driver_data*   driver_data;
	unsigned int        *mmio;

	driver_data = spi4_data[slot];
	mmio = driver_data->mmio;

	if(!(PORT_INIT(driver_data->cfg_flag)))
		return SPI4_CONFIG_PDE_SUCCESS;

	for_each_online_cpu(i) {
		cpu = cpu_logical_map(i);
		bkt = ((cpu >> 2)<<3)|(cpu & 0x03);
		bucket_map |= (1ULL << bkt);
	}

	if(pcfg->config_pde) {
		netlogic_write_reg(mmio, R_PDE_CLASS_0, (bucket_map & 0xffffffff));
		netlogic_write_reg(mmio, R_PDE_CLASS_0+1, 
				((bucket_map>>NLM_SPI4_MAX_THREADS) & 0xffffffff));

		netlogic_write_reg(mmio, R_PDE_CLASS_1, (bucket_map & 0xffffffff));
		netlogic_write_reg(mmio, R_PDE_CLASS_1+1, 
				((bucket_map>>NLM_SPI4_MAX_THREADS) & 0xffffffff));

		netlogic_write_reg(mmio, R_PDE_CLASS_2, (bucket_map & 0xffffffff));
		netlogic_write_reg(mmio, R_PDE_CLASS_2+1,
			 ((bucket_map>>NLM_SPI4_MAX_THREADS) & 0xffffffff));

		netlogic_write_reg(mmio, R_PDE_CLASS_3, (bucket_map & 0xffffffff));
		netlogic_write_reg(mmio, R_PDE_CLASS_3+1, 
			((bucket_map>>NLM_SPI4_MAX_THREADS) & 0xffffffff));
	}


	bucket = pcfg->bucket;
	credit = pcfg->credit;

	if(slot == SPI4_0){
		for(i=0;i<XLR_MAX_SPI4_CHANNEL;i++){
			netlogic_write_reg(mmio, R_XGS_TX0_BUCKET_SIZE+i, 
			bucket[MSGRNG_STNID_XGS0_TX+i]);
		}

		netlogic_write_reg(mmio, R_XGS_RFR_BUCKET_SIZE, 
			bucket[MSGRNG_STNID_XMAC0RFR]);

	}
	else if(slot == SPI4_1){
		for(i=0;i<XLR_MAX_SPI4_CHANNEL;i++)
			netlogic_write_reg(mmio, R_XGS_TX0_BUCKET_SIZE+i, 
				bucket[MSGRNG_STNID_XGS1_TX+i]);

		netlogic_write_reg(mmio, R_XGS_RFR_BUCKET_SIZE, 
			bucket[MSGRNG_STNID_XMAC1RFR]);

	}
	for(i=0;i<128;i++){
		netlogic_write_reg(mmio, R_CC_CPU0_0 + i, 
		credit->counters[i>>3][i&0x07]);
	}

	return SPI4_CONFIG_PDE_SUCCESS;
}

static void spi4_free_spill_memory(spi4_driver_data*   driver_data)
{
	if(driver_data->frin_spill != NULL)
		kfree(driver_data->frin_spill);
	
	if(driver_data->frout_spill != NULL)
		kfree(driver_data->frout_spill);
	
	if(driver_data->class_0_spill != NULL)
		kfree(driver_data->class_0_spill);

	if(driver_data->class_1_spill != NULL)
		kfree(driver_data->class_1_spill);

	if(driver_data->class_2_spill != NULL)
		kfree(driver_data->class_2_spill);

	if(driver_data->class_3_spill != NULL)
		kfree(driver_data->class_3_spill);

	return;
}	

static void* config_spill(unsigned int *mmio, int reg_start_0,
		int 	reg_start_1,
		int 	reg_size, 
		int 	size,
		void**	spill_orig)
{

	__u32 spill_size = CACHELINE_ALIGNED_ADDR(size);
	void *spill = os_cacheline_aligned_kmalloc(spill_size);
	__u64 phys_addr = 0;

	if (!spill) {
		return NULL;
	}
	*spill_orig = spill;
	phys_addr = virt_to_phys(spill);
	netlogic_write_reg(mmio, reg_start_0, ((phys_addr >> 5) & 0xffffffff));

	netlogic_write_reg(mmio, reg_start_1, ((phys_addr >> 37) & 0x07));

	netlogic_write_reg(mmio, reg_size, spill_size);

	return spill;
}

/*******************************************************************************
* Function name :       spi4_configure_spill_memory
* Input         :
* Description   :       This function programs the freein, free out and class0, 
*	class1, class2 spill memory
* RETURNS       :       void
*******************************************************************************/

static int spi4_configure_spill_memory(uint32 slot)
{

	spi4_driver_data*   driver_data;
	unsigned int        *mmio;

	driver_data = spi4_data[slot];
	mmio = driver_data->mmio;

	if(!(MSGRNG_OWN(driver_data->cfg_flag)))
		return SPI4_CONFIG_SPILL_SUCCESS;

	if(config_spill(mmio,R_REG_FRIN_SPILL_MEM_START_0,
			R_REG_FRIN_SPILL_MEM_START_1,R_REG_FRIN_SPILL_MEM_SIZE,
			MAX_FRIN_SPILL * sizeof(struct fr_desc),
			&driver_data->frin_spill) == NULL){
		return SPI4_CONFIG_SPILL_FAIL ;
	}

	if(config_spill(mmio, R_FROUT_SPILL_MEM_START_0,
			R_FROUT_SPILL_MEM_START_1,
			R_FROUT_SPILL_MEM_SIZE,
			(2* MAX_FROUT_SPILL * sizeof(struct fr_desc)),
			&driver_data->frout_spill) == NULL){
			spi4_free_spill_memory(driver_data);
			return SPI4_CONFIG_SPILL_FAIL ;	
	}

	if(config_spill(mmio, R_CLASS0_SPILL_MEM_START_0,
			R_CLASS0_SPILL_MEM_START_1,
			R_CLASS0_SPILL_MEM_SIZE,
			MAX_CLASS_0_SPILL * sizeof(union rx_tx_desc),
			&driver_data->class_0_spill) == NULL){
			spi4_free_spill_memory(driver_data);
			return SPI4_CONFIG_SPILL_FAIL;
	}

	if(config_spill(mmio,R_CLASS1_SPILL_MEM_START_0,
			R_CLASS1_SPILL_MEM_START_1,
			R_CLASS1_SPILL_MEM_SIZE,
			MAX_CLASS_1_SPILL * sizeof(union rx_tx_desc),
			&driver_data->class_1_spill) == NULL){
			spi4_free_spill_memory(driver_data);
			return SPI4_CONFIG_SPILL_FAIL ;
	}


	if(config_spill(mmio,R_CLASS2_SPILL_MEM_START_0,
			R_CLASS2_SPILL_MEM_START_1,
			R_CLASS2_SPILL_MEM_SIZE,
			MAX_CLASS_2_SPILL * sizeof(union rx_tx_desc),
			&driver_data->class_2_spill) == NULL){
			spi4_free_spill_memory(driver_data);
			return SPI4_CONFIG_SPILL_FAIL ;
	}

	if( config_spill(mmio, R_CLASS3_SPILL_MEM_START_0,
			R_CLASS3_SPILL_MEM_START_1,
			R_CLASS3_SPILL_MEM_SIZE,
			MAX_CLASS_3_SPILL * sizeof(union rx_tx_desc),
			&driver_data->class_3_spill) == NULL){
			spi4_free_spill_memory(driver_data);
			return SPI4_CONFIG_SPILL_FAIL ;
	}

	return SPI4_CONFIG_SPILL_SUCCESS;
}//end of spi4_configure_spill_memory


/*******************************************************************************
*
* Function name :       spi4_register_msgrng_handler
* Input         :       spi4 slot
* Description   :       This function will register message ring handler for the 
*       given spi4. Registered function will be called whenever msg will be send 
*	by the spi4. 
* RETURNS	: 	int
*			1 - fail
*			0 - success
*******************************************************************************/

static int spi4_register_msgrng_handler(uint32 slot)
{
	spi4_driver_data*   driver_data;
	driver_data = spi4_data[slot];
	
	if(slot == SPI4_0){
		if (register_msgring_handler(TX_STN_XGS_0, 
		spi4_msgring_handler, NULL)) {
			return SPI4_REGISTER_MSGRING_FAIL;
		}
	}	
	else if(slot == SPI4_1){
		if (register_msgring_handler(TX_STN_XGS_1, 
		spi4_msgring_handler, NULL)) {
			return SPI4_REGISTER_MSGRING_FAIL;
		}
	}
	else{
		/*invalid spi4*/
		return SPI4_SLOT_ERROR;
	}

	return SPI4_REGISTER_MSGRING_SUCESS;
}// end of spi4_register_msgrng_handler()

static int spi4_validate_config_params(void)
{
	if(XLR_TOTAL_CHANNELS > XLR_MAX_SPI4_CHANNEL){
		printk("invalid total channels\n");
		return SPI4_CALENDER_LEN_ERROR ;
	}
	if(XLR_SPI4_TX_MAXBURST1 > XLR_MAX_TX_BURST){
		printk("invalid TX max burst1\n");
		return SPI4_TX_MAXBURST1_ERROR;
	}
	if(XLR_SPI4_TX_MAXBURST2 > XLR_MAX_TX_BURST){
		printk("invalid TX max burst2\n");
		return SPI4_TX_MAXBURST2_ERROR;
	}
	if(XLR_SPI4_RX_MAXBURST1 > XLR_MAX_RX_BURST){
		printk("invalid RX max burst1\n");
		return SPI4_RX_MAXBURST1_ERROR;
	}
	if(XLR_SPI4_RX_MAXBURST2 > XLR_MAX_RX_BURST){
		printk("invalid RX max burst2\n");
		return SPI4_RX_MAXBURST2_ERROR;
	}
	if(XLR_SPI4_TX_MAXBURST2 < XLR_SPI4_TX_MAXBURST1){
		printk("invalid TX max bursts\n");
		return SPI4_TX_MAX_BURST_ERROR;
	}
	if(XLR_SPI4_RX_MAXBURST2 < XLR_SPI4_RX_MAXBURST1){
		printk("invalid RX max bursts\n");
		return SPI4_RX_MAX_BURST_ERROR ;
	}
	if(SPI4_BYTE_OFFSET > XLR_MAX_SPI4_BYTE_OFFSET){
		printk("invalid byte offset\n");
		return SPI4_BYTE_OFFSET_ERROR;
	}
	return SPI4_PARAMS_VALID;
}	

void spi4_disable_tx_rx(unsigned int	*mmio)
{
	uint32  reg_val;
	
	reg_val = netlogic_read_reg(mmio, SPI4_CNTRL_REG);
        reg_val &= ~(0xa);
        netlogic_write_reg(mmio, SPI4_CNTRL_REG, reg_val ) ;

	return;
}	

void spi4_enable_tx_rx(unsigned int	*mmio)
{
	uint32  reg_val;
	
	reg_val = netlogic_read_reg(mmio, SPI4_CNTRL_REG);
        reg_val |= 0xa;
        netlogic_write_reg(mmio, SPI4_CNTRL_REG, reg_val ) ;

	return;
}

/*******************************************************************************
*
* Function name :       spi4_init
* Input         :       spi4 slot, callback function
* Description   :       This function will initialize the given spi4. It resets 
*	the spi4, programs the tx calender, max burst, DMA and spi4 channels 
*	and FIFO. 
* RETURNS: void
*******************************************************************************/

unsigned int spi4_init(uint32 	slot, spi4_callback_func calbk_func )
{
	uint32	reg_val, byte_offset=0;
	int i;
	unsigned int 			*mmio, spi4_ret_value;
	spi4_driver_data*		driver_data;
	int tx_fifo_base[XLR_MAX_SPI4_CHANNEL];
	int tx_fifo_size[XLR_MAX_SPI4_CHANNEL];
	int rx_fifo_base[XLR_MAX_SPI4_CHANNEL];
	int rx_fifo_size[XLR_MAX_SPI4_CHANNEL];
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;
	struct port_cfg *port_cfg;

	port_cfg = &net_cfg->xgs_port[slot];
	/* if support for loading apps on same core as Linux is enabled */
	if(port_cfg->cfg_flag == 0)
		return -EINVAL;

	if((slot < SPI4_0) || (slot > SPI4_1)){
		return SPI4_SLOT_ERROR;
	}
	driver_data = (spi4_driver_data*) os_malloc(sizeof(spi4_driver_data));
	
	if(driver_data == NULL){
		return SPI4_MALLOC_FAIL;
	}
	driver_data->frin_spill =driver_data->frout_spill = NULL; 
	driver_data->class_0_spill = driver_data->class_1_spill = NULL;
	driver_data->class_2_spill = driver_data->class_3_spill = NULL; 

	spi4_data[slot] = driver_data;


	/*initilizing the spi4 and elements of the driver data*/
	driver_data->calbk_func  = calbk_func;
	driver_data->spi4_slot   = slot;
	driver_data->tx_calendar = driver_data->rx_calendar = XLR_TOTAL_CHANNELS;
	driver_data->tx_cal_sequence = TX_CAL_SEQ;
	driver_data->rx_cal_sequence = RX_CAL_SEQ;	
	driver_data->tx_maxburst1 = XLR_SPI4_TX_MAXBURST1;
	driver_data->tx_maxburst2 = XLR_SPI4_TX_MAXBURST2;
	driver_data->rx_maxburst1 = XLR_SPI4_RX_MAXBURST1;
	driver_data->rx_maxburst2 = XLR_SPI4_RX_MAXBURST2;

	driver_data->cfg_flag = port_cfg->cfg_flag;
	driver_data->mmio = (uint32_t *)port_cfg->mmio_addr;
	mmio = driver_data->mmio;

	if(PORT_INIT(driver_data->cfg_flag)) {
		spi4_ret_value = spi4_validate_config_params();
		if(spi4_ret_value != SPI4_PARAMS_VALID){
			printk("invalid configuration parameters\n");
			return spi4_ret_value ;
		}

		/*configuring the IP registers*/
		reg_val = netlogic_read_reg(mmio, SPI4_CNTRL_REG);
		reg_val &= ~(0xa);
		netlogic_write_reg(mmio, SPI4_CNTRL_REG, reg_val ) ; 

		/*programming the Hungry thresholds*/
		netlogic_write_reg(mmio, R_SPIHNGY0, 0x04040404);
		netlogic_write_reg(mmio, R_SPIHNGY1, 0x04040404 );
		netlogic_write_reg(mmio, R_SPIHNGY2, 0x04040404);
		netlogic_write_reg(mmio, R_SPIHNGY3, 0x04040404);

		/*programming the starving thresholds*/
		netlogic_write_reg(mmio, R_SPISTRV0, 0x06060606);
		netlogic_write_reg(mmio, R_SPISTRV1, 0x06060606);
		netlogic_write_reg(mmio, R_SPISTRV2, 0x06060606);
		netlogic_write_reg(mmio, R_SPISTRV3, 0x06060606);

		netlogic_write_reg(mmio, 0x50, 4);/* F_ALPHA */
		/*programming the DMA credits*/
		netlogic_write_reg(mmio, R_DMACR0, 0xffffffff);
		netlogic_write_reg(mmio, R_DMACR1, 0xffffffff);
		netlogic_write_reg(mmio, R_DMACR2, 0xffffffff);
		netlogic_write_reg(mmio, R_DMACR3, 
			(0xff<<24)|(2<<21)|(2<<18)|(2<<12)|(2<<9)|(2<<6)|(2<<3)|(2<<0));

		vtss_nlm_init(slot+1);

		for(i=0; i<8; i++){
			byte_offset <<= 3 ;
			byte_offset |= SPI4_BYTE_OFFSET ; 	
		}
 		netlogic_write_reg(mmio,DESC_PKT_CTRL_1,byte_offset);
	 	netlogic_write_reg(mmio,DESC_PKT_CTRL_2,byte_offset);
		netlogic_write_reg(mmio, PAD_CALIB_0,0x02030);


	  /*configure: TX cal, RX cal and cal seq*/
	  netlogic_write_reg(mmio, SPI4_TX_CAL_LEN, driver_data->tx_calendar);
	  netlogic_write_reg(mmio, SPI4_RX_CAL_LEN, driver_data->rx_calendar);
	  netlogic_write_reg(mmio, SPI4_TX_CAL_MAX , driver_data->tx_cal_sequence);
	  netlogic_write_reg(mmio, SPI4_RX_CAL_MAX, driver_data->rx_cal_sequence);


		for(i=0;i< driver_data->tx_calendar; i++){
		  	netlogic_write_reg(mmio,SPI4_TX_CAL_X, (i | (i<<16)));
		}
		for(i=0;i< driver_data->rx_calendar; i++){
		  	netlogic_write_reg(mmio,SPI4_RX_CAL_X, (i | (i<<16)));
		}
	}
	
	if(PORT_EN(driver_data->cfg_flag)) {

		netlogic_write_reg(mmio, R_TX_CONTROL,
      		((1<<W_TX_CONTROL__TxThreshold) | TX_THRESHOLD_SIZE));

		netlogic_write_reg(mmio, R_RX_CONTROL, 1);

	}

	if(PORT_INIT(driver_data->cfg_flag)) {
		netlogic_write_reg(mmio, R_DESC_PACK_CTRL,
		    ((4 << 20) | REG_FRAME_SIZE));

		netlogic_write_reg(mmio, R_L2ALLOCCTRL, 0x2);

		mdelay(5);

		netlogic_write_reg(mmio, SPI4_INTR_REG, CLEAR_INT);
		netlogic_write_reg(mmio, SPI4_TX_STATUS , CLEAR_TX_STATUS);
		netlogic_write_reg(mmio, SPI4_RX_STATUS, CLEAR_RX_STATUS);


		/* change rx maxburst 1 */
		for (i=0;i<XLR_MAX_SPI4_CHANNEL;i++)
    		netlogic_write_reg(mmio, SPI4_RX_MAXBURST1_I,
        		XLR_SPI4_RX_MAXBURST1 + (i << XLR_MAX_SPI4_CHANNEL));


	  	/* change rx maxburst 2 */
		for (i=0;i<XLR_MAX_SPI4_CHANNEL;i++)
		    netlogic_write_reg(mmio, SPI4_RX_MAXBURST2_I,
        		XLR_SPI4_RX_MAXBURST2 + (i << XLR_MAX_SPI4_CHANNEL));

		for (i=0;i<XLR_MAX_SPI4_CHANNEL;i++){
    		tx_fifo_base[i]  = i*8;  
		    tx_fifo_size[i]  = 8 ; 
    		rx_fifo_base[i]  = i*32;  
	    	rx_fifo_size[i]  = 32 ; 
  		}


		for (i=0;i<XLR_MAX_SPI4_CHANNEL;i++){
    		netlogic_write_reg(mmio, SPI4_TX_FIFO_BASE_I,
      				tx_fifo_base[i] + (i<<XLR_MAX_SPI4_CHANNEL));
		    netlogic_write_reg(mmio, SPI4_RX_FIFO_BASE_I,
      				rx_fifo_base[i] + (i<<XLR_MAX_SPI4_CHANNEL));
  		}

	 	for (i=0;i<XLR_MAX_SPI4_CHANNEL;i++){
    		netlogic_write_reg(mmio, SPI4_TX_FIFO_DEPTH_I,
		   		tx_fifo_size[i] + (i<< XLR_MAX_SPI4_CHANNEL));
		    netlogic_write_reg(mmio, SPI4_RX_FIFO_DEPTH_I,
      			rx_fifo_size[i] + (i<< XLR_MAX_SPI4_CHANNEL));
  		}
	}

	if(PORT_EN(driver_data->cfg_flag)) {
		netlogic_write_reg(mmio, 0x78, 4);
		netlogic_write_reg(mmio, SPI4_CNTRL_REG, 0x00000e0f);

		mdelay(10);

		i = netlogic_read_reg(mmio, SPI4_TX_STATUS);
		if(!(i & SPI4_TX_STATUS_TX_SYNC)){
			spi4_disable_tx_rx(mmio);
			printk("TX path no sync\n");
			return SPI4_TX_SYNC_FAIL;
		}

		i = netlogic_read_reg(mmio, SPI4_RX_STATUS);
		if(!(i & SPI4_RX_STATUS_RX_SYNC)){
			spi4_disable_tx_rx(mmio);
			printk("RX path no sync\n");
			return SPI4_RX_SYNC_FAIL;
		}
	}
	
	spi4_ret_value = spi4_configure_spill_memory(slot);
	if(spi4_ret_value != SPI4_CONFIG_SPILL_SUCCESS){
			spi4_disable_tx_rx(mmio);
			printk("spill memory configuration failed\n");
			return spi4_ret_value;
	}

	spi4_ret_value = spi4_configure_pde_spray_mode(slot, port_cfg);
	if(spi4_ret_value != SPI4_CONFIG_PDE_SUCCESS){
			spi4_disable_tx_rx(mmio);
			spi4_free_spill_memory(driver_data);
			printk("pde configuration failed\n");
			return spi4_ret_value ;
	}

//	if(PORT_INIT(driver_data->cfg_flag)) 
//		rmik_config_pde(TYPE_SPI4, slot, mmio);

	spi4_ret_value = spi4_register_msgrng_handler(slot);
	if(spi4_ret_value != SPI4_REGISTER_MSGRING_SUCESS){
		spi4_disable_tx_rx(mmio);
		spi4_free_spill_memory(driver_data);
		printk("registering msgring handler failed\n");
		return spi4_ret_value ;
	}

	return SPI4_INIT_SUCCESS;

}// end of function spi4_init()
/*******************************************************************************
* Function name :       spi4_msgring_handler
* Input         :       
* Description   :       This function will be called when spi4 sends any msg. 
*	It handles TX_DONE and RX_IND mesg and informs the same to registered
*	function by the upper application.
* RETURNS       :       void
*******************************************************************************/
static void spi4_msgring_handler(int 	bucket, 	int size, 
			int	code, 		int stid,
			struct 	msgrng_msg *msg, void *data)
{

	unsigned int slot, port, ctrl, length;
	unsigned long addr = 0;
	unsigned int  error=0, th_id;
	char*	ptr;
	spi4_driver_data*   driver_data;


	if(stid == MSGRNG_STNID_XGS0FR)
		slot = SPI4_0;
	else if(stid == MSGRNG_STNID_XGS1FR)
		slot = SPI4_1;
	else{
		printk("ERROR: wrong slot\n");
		return;
	}
	driver_data = spi4_data[slot];
	port = get_port(msg->msg0);



	length = get_length(msg->msg0);
  if(length == 0)
    ctrl = CTRL_REG_FREE;
  else
	  ctrl = CTRL_SNGL;


	if (ctrl == CTRL_REG_FREE ) {
   	/*TX complete*/
    		addr = msg->msg0 & 0xffffffffffULL;
    		addr = (unsigned long) phys_to_virt(addr);
		ptr = (char*) addr;
   	(*driver_data->calbk_func)
      (SPI4_TX_DONE,slot,bucket,ptr,length, error);
	}	
	else if(ctrl == CTRL_SNGL || ctrl == CTRL_START){
		/*RX indication*/
		addr = (unsigned long) bus_to_virt(get_address(msg->msg0));
		ptr = (char*) addr;
		error = ((msg->msg0 >> 62)& 0x01);

		if((port >= XLR_TOTAL_CHANNELS)){
			/*if wrong port is received, then treat that as error packet
			and try to replenish it*/
			error = 1; 
		}
		
		if(error){
			spi4_program_rx_desc(slot,ptr);
			th_id = hard_smp_processor_id();
			g_dip4_error[th_id]++;
		}
		else{
		length -=  (SPI4_BYTE_OFFSET + MAC_CRC_LEN );
		(*driver_data->calbk_func)
			(SPI4_RX_IND,slot,port,ptr,length, error);
		}
	}
	return;
}//spi4_msgring_handler()

/*******************************************************************************
* Function name :       spi4_tx
* Input         :
* Description   :       This function will be called by the upper application
*	to transmit data. Before doing TX it makes sure TX and RX path are in 
*	sync.
* RETURNS       :       int
*			1 - fail
*			0 - success
*******************************************************************************/

int spi4_tx(	unsigned int thr_id, uint32 slot,  uint32 spi4_port, 
						char* data, unsigned char* skb,uint32 len)
{
	unsigned int 		msgrng_flags;
	spi4_driver_data*   	driver_data;
	struct msgrng_msg 	msg;
	int 			stid=0, ret =0;


	driver_data = spi4_data[slot];

	stid = spi4_make_desc_tx(thr_id, &msg,  driver_data->spi4_slot,
													spi4_port, TYPE_SPI4, virt_to_phys(data), 
													(unsigned long) skb, len);

	__sync();
	msgrng_access_enable(msgrng_flags);
  if (message_send_retry(2, MSGRNG_CODE_SPI4, stid, &msg)){
    ret =  SPI4_TX_FAIL;
	}
	msgrng_access_disable(msgrng_flags);
	return ret;
}// end of spi4_tx()

/*******************************************************************************
* Function name :       spi4_program_rx_desc
* Input         :
* Description   :       This function will make a regular free descriptor 
*	and sends it to spi4.
* RETURNS       :       int
*                       1 - fail
*                       0 - success
*******************************************************************************/

void spi4_program_rx_desc(uint32 	slot, 	
			char*		addr)
{

	unsigned long 		msgrng_flags;
	int 			stid = 0;
	struct msgrng_msg 	msg;
	spi4_driver_data*   	driver_data;

	driver_data = spi4_data[slot];

	stid = spi4_make_desc_rfr(&msg, driver_data->spi4_slot,
				TYPE_SPI4 , virt_to_bus(addr));
	__sync();
	msgrng_access_enable(msgrng_flags);
	while (message_send(1, MSGRNG_CODE_SPI4, stid, &msg));
	msgrng_access_disable(msgrng_flags);


	return ;	
}


/*******************************************************************************
* Function name :       spi4_open
* Input         :
* Description   :       This function enable TX and RX of the spi4 also sends 
*	a jumbo frame.
* RETURNS       :       void
*******************************************************************************/

int spi4_open(uint32 slot)
{

	spi4_driver_data*	driver_data;
	unsigned int        	*mmio;

	driver_data = spi4_data[slot];
	if(driver_data == NULL){
		return  SPI4_SLOT_ERROR;
	}

	mmio = driver_data->mmio;
	if(mmio == NULL){
		return SPI4_MMIO_ERROR;
	}

		return SPI4_OPEN_SUCCESS;
}// end of function spi4_open()

/*******************************************************************************
* Function name :       spi4_close
* Input         :
* Description   :       This function disable TX and RX of the spi4 .
* RETURNS       :       void
*******************************************************************************/

void spi4_close(uint32 slot)
{

	spi4_driver_data*   	driver_data;
	unsigned int        	*mmio;
	unsigned int 		tmp;

	driver_data = spi4_data[slot];
	mmio = driver_data->mmio;

	tmp = netlogic_read_reg(mmio, R_TX_CONTROL);
	tmp &= ~(1<<W_TX_CONTROL__TxThreshold);
	netlogic_write_reg(mmio, R_TX_CONTROL, tmp);

	tmp = netlogic_read_reg(mmio, R_RX_CONTROL);
	tmp &= ~(1<<O_RX_CONTROL__RxEnable) ;
	netlogic_write_reg(mmio, R_RX_CONTROL, tmp);

	tmp = netlogic_read_reg(mmio, SPI4_CNTRL_REG);
	tmp &= ~(TX_ENABLE |  RX_ENABLE);
	netlogic_write_reg(mmio, SPI4_CNTRL_REG, tmp);

	return;

}

int spi4_read_reg(uint32 slot, uint32 addr)
{

	spi4_driver_data*   driver_data;
	unsigned int        *mmio;
	unsigned int        tmp;

	driver_data = spi4_data[slot];
	mmio = driver_data->mmio;

	tmp = netlogic_read_reg(mmio, addr);
	return tmp;
}


