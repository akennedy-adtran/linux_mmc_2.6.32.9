#ifndef andy_h
#define andy_h

#if 1
#define andy_attention "==========================================  "

#define andy_print(args...) do { mdelay(2) ; printk(args) ; } while (0)

#define printm()			andy_print(andy_attention"MARK %5i %-30s %-40s\n", __LINE__, __FILE__, __func__)
#define printg()			andy_print(andy_attention"GOOD %5i %-30s %-40s Returning 0\n", __LINE__, __FILE__, __func__)
#define printb(retval)			andy_print(andy_attention"BAD  %5i %-30s %-40s Returning %s (0x%lx)\n", __LINE__, __FILE__, __func__, #retval, (int)(retval))
#define printv(retval, cfg_string...)	andy_print(andy_attention"VAL  %5i %-30s %-40s Returning "cfg_string"%s (0x%lx)\n", __LINE__, __FILE__, __func__, #retval, (int)(retval))
#define prints(cfg_string, ...)		andy_print(andy_attention"SPCL %5i %-30s %-40s " cfg_string "\n", __LINE__, __FILE__, __func__, ##__VA_ARGS__)
#else
#define printm()

#define printg()

#define printb(retval)

#define printv(retval, cfg_string...)

#define prints(cfg_string, ...)
#endif

#endif
