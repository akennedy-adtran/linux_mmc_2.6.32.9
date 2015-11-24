/*-
 * Copyright 2005-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * drivers/net/kgdboe.c
 *
 * A network interface for GDB.
 * Based upon 'gdbserial' by David Grothe <dave@gcom.com>
 * and Scott Foehner <sfoehner@engr.sgi.com>
 *
 * Maintainers: Amit S. Kale <amitkale@linsyssoft.com> and
 * 		Tom Rini <trini@kernel.crashing.org>
 *
 * 2004 (c) Amit S. Kale <amitkale@linsyssoft.com>
 * 2004-2005 (c) MontaVista Software, Inc.
 * 2005 (c) Wind River Systems, Inc.
 *
 * Other folks:
 * San Mehat <nettwerk@biodome.org>
 * Robert Walsh <rjwalsh@durables.org>
 * wangdi <wangdi@clusterfs.com>.
 * Matt Mackall <mpm@selenic.com>
 * Pavel Machek <pavel@suse.cz>
 * Jason Wessel <jason.wessel@windriver.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Changes:
 * 3/10/05 - Jason Wessel <jason.wessel@windriver.com>
 * - Added ability to compile/load as module
 *
 * Known problems:
 * - There is no way to deny the unloading of the module
 *   if KGDB is connected, but an attempt is made to handle it
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/kgdb.h>
#include <linux/netpoll.h>
#include <linux/init.h>

#include <asm/atomic.h>

#define IN_BUF_SIZE 512		/* power of 2, please */
#define OUT_BUF_SIZE 30		/* We don't want to send too big of a packet. */

static char in_buf[IN_BUF_SIZE], out_buf[OUT_BUF_SIZE];
static int in_head, in_tail, out_count;
static atomic_t in_count;
/* 0 = unconfigured, 1 = netpoll options parsed, 2 = fully configured. */
static int configured;

static void rx_hook(struct netpoll *np, int port, char *msg, int len);
static void eth_pre_exception_handler(void);
static void eth_post_exception_handler(void);
static int eth_get_char(void);
static void eth_flush_buf(void);
static void eth_put_char(int chr);
int init_kgdboe(void);

static struct netpoll np = {
	.name = "kgdboe",
	.dev_name = "eth0",
	.rx_hook = rx_hook,
	.local_port = 6443,
	.remote_port = 6442,
	.remote_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};

MODULE_DESCRIPTION("KGDB driver for network interfaces");
MODULE_LICENSE("GPL");
static char config[256];
module_param_string(kgdboe, config, 256, 0);
MODULE_PARM_DESC(kgdboe, " kgdboe=[src-port]@[src-ip]/[dev],"
		 "[tgt-port]@<tgt-ip>/<tgt-macaddr>\n");

static struct kgdb_io local_kgdb_io_ops = {
	.read_char = eth_get_char,
	.write_char = eth_put_char,
	.init = init_kgdboe,
	.flush = eth_flush_buf,
	.pre_exception = eth_pre_exception_handler,
	.post_exception = eth_post_exception_handler
};

static void eth_pre_exception_handler(void)
{
	netpoll_set_trap(1);
}

static void eth_post_exception_handler(void)
{
	netpoll_set_trap(0);
}

static int eth_get_char(void)
{
	int chr;

	while (atomic_read(&in_count) == 0)
		netpoll_poll(&np);

	chr = in_buf[in_tail++];
	in_tail &= (IN_BUF_SIZE - 1);
	atomic_dec(&in_count);
	return chr;
}

static void eth_flush_buf(void)
{
	if (out_count && np.dev) {
		netpoll_send_udp(&np, out_buf, out_count);
		memset(out_buf, 0, sizeof(out_buf));
		out_count = 0;
	}
}

static void eth_put_char(int chr)
{
	out_buf[out_count++] = chr;
	if (out_count == OUT_BUF_SIZE)
		eth_flush_buf();
}

static void rx_hook(struct netpoll *np, int port, char *msg, int len)
{
	int i;

	np->remote_port = port;

	/*
	 * This could be GDB trying to attach.  But it could also be GDB
	 * finishing up a session, with kgdb_connected=0 but GDB sending
	 * an ACK for the final packet.  To make sure we don't try and
	 * make a breakpoint when GDB is leaving, make sure that if
	 * !kgdb_connected the only len == 1 packet we allow is ^C.
	 */
	if (!kgdb_connected && (len != 1 || msg[0] == 3) &&
	    !atomic_read(&kgdb_setting_breakpoint))
		tasklet_schedule(&kgdb_tasklet_breakpoint);

	for (i = 0; i < len; i++) {
		if (msg[i] == 3)
			tasklet_schedule(&kgdb_tasklet_breakpoint);

		if (atomic_read(&in_count) >= IN_BUF_SIZE) {
			/* buffer overflow, clear it */
			in_head = in_tail = 0;
			atomic_set(&in_count, 0);
			break;
		}
		in_buf[in_head++] = msg[i];
		in_head &= (IN_BUF_SIZE - 1);
		atomic_inc(&in_count);
	}
}

static int option_setup(char *opt)
{
	configured = !netpoll_parse_options(&np, opt);
	return 0;
}

__setup("kgdboe=", option_setup);

int init_kgdboe(void)
{
	/* Already done? */
	if (configured == 2)
		return 0;

	if (strlen(config))
		option_setup(config);

	if (!configured) {
		printk("kgdboe: configuration incorrect - kgdboe not "
		       "loaded.\n");
		printk("  Usage: kgdboe=[src-port]@[src-ip]/[dev],[tgt-port]"
		       "@<tgt-ip>/[tgt-macaddr]\n");
		return -EINVAL;
	}

	if (netpoll_setup(&np)) {
		printk("kgdboe: netpoll_setup failed kgdboe failed\n");
		return -EINVAL;
	}

	if (kgdb_register_io_module(&local_kgdb_io_ops))
		return -EINVAL;

	printk(KERN_INFO "kgdboe: debugging over ethernet enabled\n");

	configured = 2;

	return 0;
}

static void cleanup_kgdboe(void)
{
	netpoll_cleanup(&np);
	configured = 0;

	kgdb_unregister_io_module(&local_kgdb_io_ops);
}

module_init(init_kgdboe);
module_exit(cleanup_kgdboe);
