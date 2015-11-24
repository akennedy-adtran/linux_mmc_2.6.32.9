Steps for running Linux TCP/IP Routing performance benchmark
------------------------------------------------------------

Overview:
When performing routing benchmark on Linux TCP/IP, we use NAPI mode to reduce
number of interrupts.

xlr_napi
~~~~~~~~
Enables NAPI polling mode on GMAC packet drivers. Option is disabled by default.
This option also enables another optimization: it sets lockless tx mode 
(NETIF_F_LLTX) on the interfaces and turns off use of the default pfifo_fast 
interface transmit queue in the stack (by setting tx_queue_len to 0). With 32
vcpus, the cost of synchronizing the transmit interface queue using the
pfifo_fast discipline is pretty high. Since the transmit of packets to the mac
on XLR is via message ring on which multiple agents can transmit message
simultaneously lockless tx helps to avoid locks in the generic driver.

XLR_NAPI option is currently incompatible with security and/or compression
applications: XLR/XLS security driver has to be disabled and kernel recompiled
 
   make menuconfig
          -->Device Drivers
                -->RMI security subsystem driver configuration
                       --> Disable RMI security subsystem driver
 
   exit saving kernel config, execute "make clean && make"
 
XLR_NAPI option requires xls_msgring_thread_mask to match user_mask_cpus:
E.g. if linux is booted with "userapp_mask_cpus 0x33333333" then linux kernel
option "xlr_msgring_thread_mask=0x03" must be passed to the kernel.

----------------------------------------------------------------
1.  As per instructions above, change the Linux kernel configuration to
    disable RMI security subsystem.
2.  Ensure that the Compression subsystem is also disabled.  It is disabled
    by default.  Check and verify it is so.
3.  make clean && make vmlinux
4.  Copy the image (vmlinux) to tftp server.
5.  ifconfig -i pci-natsemi-0
6.  tftpc -b 1468 -f pirast/vmlinux
7.  elfload
8.  userapp xlr_napi
9.  Login as root. (By default PDE is configured to round-robin packets to
    all threads)
10. ./.setuprc
11. echo 1 >  /proc/sys/net/ipv4/ip_forward
10. ifconfig eth1 192.168.10.44
11. ifconfig eth2 192.168.11.44
12. ifconfig eth3 192.168.12.44
13. ifconfig eth4 192.168.13.44

After the above setup, initiate ping from Smartbits to ensure the setup is successful.  Start ping test by going into "Port setup" and increasing the Ping frequency and clicking OK.  This will start the ping from Smartbits and responses will be recorded in the statistics.  You can view the counters by right-clicking on the Smartbits port and "Display Counters".  To stop ping, go to "Port setup" and reduce the Ping frequency to 0 and click OK.

