/* 4_2_4 */
static inline int dev_pm_domain_attach(struct device *dev, bool power_poff)
{
	return -ENODEV;
}

static inline void dev_pm_domain_detach(struct device *dev, bool power_off) {}
