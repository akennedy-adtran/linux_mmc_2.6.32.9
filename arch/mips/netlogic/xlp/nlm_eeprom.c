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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include <asm/netlogic/hal/nlm_eeprom.h>
#include <asm/netlogic/i2c-xlp.h>

/* Read/write eval board MAC address EEPROM.  Originally was i2c.c - combined with
 * old nlm_eeprom.c - but overall changed to be more like an I2C client to work
 * with the existing I2C subsystem. */

struct nlm_eeprom_data
{
	struct i2c_xlp_data *priv;
	uint8_t dev_addr;
};

static struct nlm_eeprom_data nlm_eeprom;

/* Return 1 if magic bytes match (address has been set) and update buffer.
 * Return 0 if magic bytes do not match and leave buffer unchanged.
 */
int nlm_eeprom_get_mac_addr(unsigned char *mac, int interface)
{
	unsigned char buf[MAGIC_LEN + MAC_LEN];
	uint32_t offset = (interface == 0) ? MAGIC_OFF : MAGIC2_OFF;

	xlp_i2c_read(nlm_eeprom.priv, nlm_eeprom.dev_addr, offset, 0, MAGIC_LEN + MAC_LEN, buf);

	if((buf[0] != MAGIC_BYTE0) || (buf[1] != MAGIC_BYTE1))
		return 0;

	memcpy(mac, buf + MAGIC_LEN, MAC_LEN);
	return 1;
}
EXPORT_SYMBOL(nlm_eeprom_get_mac_addr);

void nlm_eeprom_set_mac_addr(unsigned char *mac, int interface)
{
	unsigned char buff[MAGIC_LEN + MAC_LEN];
	unsigned int offset = (interface == 0) ? MAGIC_OFF : MAGIC2_OFF;

	buff[0] = MAGIC_BYTE0;
	buff[1] = MAGIC_BYTE1;

	xlp_i2c_write(nlm_eeprom.priv, nlm_eeprom.dev_addr, offset, 0, MAGIC_LEN + MAC_LEN, buff);
}
EXPORT_SYMBOL(nlm_eeprom_set_mac_addr);

void nlm_eeprom_dump(unsigned char *data, int offset, int len)
{
	if(offset < 0)
		offset = 0;
	if(offset >= NLM_EEPROM_LEN)
		offset = NLM_EEPROM_LEN - 1;
	if((offset + len) > NLM_EEPROM_LEN)
		len = NLM_EEPROM_LEN - offset;

	xlp_i2c_read(nlm_eeprom.priv, nlm_eeprom.dev_addr, offset, 0, len, data);
}
EXPORT_SYMBOL(nlm_eeprom_dump);

/* Basic driver stuff */
static const struct i2c_device_id nlm_eeprom_id[] =
{
	{ "nlm_eeprom", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nlm_eeprom_id);

static int nlm_eeprom_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char buff[NLM_EEPROM_LEN];
	int ret;

	/* Attempt to read - if this fails, we might not really be here */
	ret = xlp_i2c_read(client->adapter->algo_data, client->addr, 0, 0, NLM_EEPROM_LEN, buff);

	if(!ret) {
		nlm_eeprom.priv = client->adapter->algo_data;
		nlm_eeprom.dev_addr = client->addr;
		i2c_set_clientdata(client, &nlm_eeprom);	// Not really needed
		printk("nlm_eeprom on I2C bus %d, address 0x%x\n", client->adapter->id, client->addr);
	}
	return ret;
}

static int __devexit nlm_eeprom_remove(struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_driver nlm_eeprom_driver =
{
	.driver = {
		.name  = "nlm_eeprom",
		.owner = THIS_MODULE
	},
	.class = I2C_CLASS_SPD,			// Not an SPD but closest class there is
	.probe = nlm_eeprom_probe,
	.remove = __devexit_p(nlm_eeprom_remove),
	.id_table = nlm_eeprom_id
};

static int __init nlm_eeprom_init(void)
{
	return i2c_add_driver(&nlm_eeprom_driver);
}

static void __exit nlm_eeprom_exit(void)
{
	return i2c_del_driver(&nlm_eeprom_driver);
}

module_init(nlm_eeprom_init);
module_exit(nlm_eeprom_exit);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("XLP eval system MAC address EEPROM");
MODULE_LICENSE("GPL");
