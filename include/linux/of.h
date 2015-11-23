#ifndef _LINUX_OF_H
#define _LINUX_OF_H
/*
 * Definitions for talking to the Open Firmware PROM on
 * Power Macintosh and other computers.
 *
 * Copyright (C) 1996-2005 Paul Mackerras.
 *
 * Updates for PPC64 by Peter Bergner & David Engebretsen, IBM Corp.
 * Updates for SPARC64 by David S. Miller
 * Derived from PowerPC and Sparc prom.h files by Stephen Rothwell, IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/mod_devicetable.h>

//#include <asm/prom.h>  //ADTRAN

typedef u32 phandle;

/* flag descriptions */
#define OF_DYNAMIC	1 /* node and properties were allocated via kmalloc */
#define OF_DETACHED	2 /* node has been detached from the device tree */

#define OF_BAD_ADDR	((u64)-1)

extern struct device_node *of_find_node_by_name(struct device_node *from,
	const char *name);
#define for_each_node_by_name(dn, name) \
	for (dn = of_find_node_by_name(NULL, name); dn; \
	     dn = of_find_node_by_name(dn, name))
extern struct device_node *of_find_node_by_type(struct device_node *from,
	const char *type);
#define for_each_node_by_type(dn, type) \
	for (dn = of_find_node_by_type(NULL, type); dn; \
	     dn = of_find_node_by_type(dn, type))
extern struct device_node *of_find_compatible_node(struct device_node *from,
	const char *type, const char *compat);
#define for_each_compatible_node(dn, type, compatible) \
	for (dn = of_find_compatible_node(NULL, type, compatible); dn; \
	     dn = of_find_compatible_node(dn, type, compatible))
extern struct device_node *of_find_matching_node(struct device_node *from,
	const struct of_device_id *matches);
#define for_each_matching_node(dn, matches) \
	for (dn = of_find_matching_node(NULL, matches); dn; \
	     dn = of_find_matching_node(dn, matches))
extern struct device_node *of_find_node_by_path(const char *path);
extern struct device_node *of_find_node_by_phandle(phandle handle);
extern struct device_node *of_get_parent(const struct device_node *node);
extern struct device_node *of_get_next_parent(struct device_node *node);
extern struct device_node *of_get_next_child(const struct device_node *node,
					     struct device_node *prev);
#define for_each_child_of_node(parent, child) \
	for (child = of_get_next_child(parent, NULL); child != NULL; \
	     child = of_get_next_child(parent, child))

extern struct device_node *of_find_node_with_property(
	struct device_node *from, const char *prop_name);
#define for_each_node_with_property(dn, prop_name) \
	for (dn = of_find_node_with_property(NULL, prop_name); dn; \
	     dn = of_find_node_with_property(dn, prop_name))

#if CONFIG_OF  //ADTRAN
extern struct property *of_find_property(const struct device_node *np,
					 const char *name,
					 int *lenp);
extern int of_device_is_compatible(const struct device_node *device,
				   const char *);
#else
static inline struct property *of_find_property(const struct device_node *np,
						const char *name,
						int *lenp)
{
	return NULL;
}

static inline int of_device_is_compatible(const struct device_node *device,
					  const char *name)
{
	return 0;
}
#endif

extern int of_device_is_available(const struct device_node *device);
extern const void *of_get_property(const struct device_node *node,
				const char *name,
				int *lenp);
extern int of_n_addr_cells(struct device_node *np);
extern int of_n_size_cells(struct device_node *np);
extern const struct of_device_id *of_match_node(
	const struct of_device_id *matches, const struct device_node *node);
extern int of_modalias_node(struct device_node *node, char *modalias, int len);
extern struct device_node *of_parse_phandle(struct device_node *np,
					    const char *phandle_name,
					    int index);
extern int of_parse_phandles_with_args(struct device_node *np,
	const char *list_name, const char *cells_name, int index,
	struct device_node **out_node, const void **out_args);

// ADTRAN ADDED:
static inline void of_core_init(void)
{
}

static inline bool is_of_node(struct fwnode_handle *fwnode)
{
	return false;
}

static inline struct device_node *to_of_node(struct fwnode_handle *fwnode)
{
	return NULL;
}

static inline const char* of_node_full_name(const struct device_node *np)
{
	return "<no-node>";
}

static inline struct device_node *of_find_matching_node_and_match(
	struct device_node *from,
	const struct of_device_id *matches,
	const struct of_device_id **match)
{
	return NULL;
}

static inline struct device_node *of_find_node_opts_by_path(const char *path,
	const char **opts)
{
	return NULL;
}

static inline struct device_node *of_get_next_available_child(
	const struct device_node *node, struct device_node *prev)
{
	return NULL;
}

static inline bool of_have_populated_dt(void)
{
	return false;
}

static inline struct device_node *of_get_child_by_name(
					const struct device_node *node,
					const char *name)
{
	return NULL;
}

static inline bool of_device_is_big_endian(const struct device_node *device)
{
	return false;
}

static inline int of_property_count_elems_of_size(const struct device_node *np,
			const char *propname, int elem_size)
{
	return -ENOSYS;
}

static inline int of_property_read_u32_index(const struct device_node *np,
			const char *propname, u32 index, u32 *out_value)
{
	return -ENOSYS;
}

static inline int of_property_read_u8_array(const struct device_node *np,
			const char *propname, u8 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_u16_array(const struct device_node *np,
			const char *propname, u16 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_u32_array(const struct device_node *np,
					     const char *propname,
					     u32 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_u32(const struct device_node *np,
				       const char *propname,
				       u32 *out_value)
{
	return of_property_read_u32_array(np, propname, out_value, 1);
}

static inline int of_property_read_s32(const struct device_node *np,
				       const char *propname,
				       s32 *out_value)
{
	return of_property_read_u32(np, propname, (u32*) out_value);
}


static inline int of_property_read_u64_array(const struct device_node *np,
					     const char *propname,
					     u64 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_string(struct device_node *np,
					  const char *propname,
					  const char **out_string)
{
	return -ENOSYS;
}

static inline int of_property_read_string_helper(struct device_node *np,
						 const char *propname,
						 const char **out_strs, size_t sz, int index)
{
	return -ENOSYS;
}

static inline struct device_node *of_get_cpu_node(int cpu,
					unsigned int *thread)
{
	return NULL;
}

static inline int of_property_read_u64(const struct device_node *np,
				       const char *propname, u64 *out_value)
{
	return -ENOSYS;
}

static inline int of_property_match_string(struct device_node *np,
					   const char *propname,
					   const char *string)
{
	return -ENOSYS;
}

static inline int of_parse_phandle_with_args(struct device_node *np,
					     const char *list_name,
					     const char *cells_name,
					     int index,
					     struct of_phandle_args *out_args)
{
	return -ENOSYS;
}

static inline int of_parse_phandle_with_fixed_args(const struct device_node *np,
	const char *list_name, int cells_count, int index,
	struct of_phandle_args *out_args)
{
	return -ENOSYS;
}

static inline int of_count_phandle_with_args(struct device_node *np,
					     const char *list_name,
					     const char *cells_name)
{
	return -ENOSYS;
}

static inline int of_alias_get_id(struct device_node *np, const char *stem)
{
	return -ENOSYS;
}

static inline int of_alias_get_highest_id(const char *stem)
{
	return -ENOSYS;
}

static inline int of_machine_is_compatible(const char *compat)
{
	return 0;
}

static inline bool of_console_check(const struct device_node *dn, const char *name, int index)
{
	return false;
}

static inline const __be32 *of_prop_next_u32(struct property *prop,
		const __be32 *cur, u32 *pu)
{
	return NULL;
}

static inline const char *of_prop_next_string(struct property *prop,
		const char *cur)
{
	return NULL;
}

static inline int of_node_check_flag(struct device_node *n, unsigned long flag)
{
	return 0;
}

static inline int of_node_test_and_set_flag(struct device_node *n,
					    unsigned long flag)
{
	return 0;
}

static inline void of_node_set_flag(struct device_node *n, unsigned long flag)
{
}

static inline void of_node_clear_flag(struct device_node *n, unsigned long flag)
{
}

static inline int of_property_check_flag(struct property *p, unsigned long flag)
{
	return 0;
}

static inline void of_property_set_flag(struct property *p, unsigned long flag)
{
}

static inline void of_property_clear_flag(struct property *p, unsigned long flag)
{
}

/**
 * of_property_read_bool - Findfrom a property
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 *
 * Search for a property in a device node.
 * Returns true if the property exists false otherwise.
 */
static inline bool of_property_read_bool(const struct device_node *np,
					 const char *propname)
{
	struct property *prop = of_find_property(np, propname, NULL);

	return prop ? true : false;
}

static inline void of_node_put(struct device_node *node) { }

#define of_match_ptr(_ptr)	NULL

#endif /* _LINUX_OF_H */
