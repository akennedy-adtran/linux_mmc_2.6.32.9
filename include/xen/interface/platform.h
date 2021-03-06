/******************************************************************************
 * platform.h
 *
 * Hardware platform operations. Intended for use by domain-0 kernel.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright (c) 2002-2006, K Fraser
 */

#ifndef __XEN_PUBLIC_PLATFORM_H__
#define __XEN_PUBLIC_PLATFORM_H__

#include "xen.h"

#define XENPF_INTERFACE_VERSION 0x03000001

/*
 * Set clock such that it would read <secs,nsecs> after 00:00:00 UTC,
 * 1 January, 1970 if the current system time was <system_time>.
 */
#define XENPF_settime             17
struct xenpf_settime {
    /* IN variables. */
    uint32_t secs;
    uint32_t nsecs;
    uint64_t system_time;
};
typedef struct xenpf_settime xenpf_settime_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_settime_t);

/*
 * Request memory range (@mfn, @mfn+@nr_mfns-1) to have type @type.
 * On x86, @type is an architecture-defined MTRR memory type.
 * On success, returns the MTRR that was used (@reg) and a handle that can
 * be passed to XENPF_DEL_MEMTYPE to accurately tear down the new setting.
 * (x86-specific).
 */
#define XENPF_add_memtype         31
struct xenpf_add_memtype {
    /* IN variables. */
    unsigned long mfn;
    uint64_t nr_mfns;
    uint32_t type;
    /* OUT variables. */
    uint32_t handle;
    uint32_t reg;
};
typedef struct xenpf_add_memtype xenpf_add_memtype_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_add_memtype_t);

/*
 * Tear down an existing memory-range type. If @handle is remembered then it
 * should be passed in to accurately tear down the correct setting (in case
 * of overlapping memory regions with differing types). If it is not known
 * then @handle should be set to zero. In all cases @reg must be set.
 * (x86-specific).
 */
#define XENPF_del_memtype         32
struct xenpf_del_memtype {
    /* IN variables. */
    uint32_t handle;
    uint32_t reg;
};
typedef struct xenpf_del_memtype xenpf_del_memtype_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_del_memtype_t);

/* Read current type of an MTRR (x86-specific). */
#define XENPF_read_memtype        33
struct xenpf_read_memtype {
    /* IN variables. */
    uint32_t reg;
    /* OUT variables. */
    unsigned long mfn;
    uint64_t nr_mfns;
    uint32_t type;
};
typedef struct xenpf_read_memtype xenpf_read_memtype_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_read_memtype_t);

#define XENPF_microcode_update    35
__DEFINE_GUEST_HANDLE(void, void);
__DEFINE_GUEST_HANDLE(uchar, unsigned char);
struct xenpf_microcode_update {
    /* IN variables. */
    GUEST_HANDLE(void) data;          /* Pointer to microcode data */
    uint32_t length;                  /* Length of microcode data. */
};
typedef struct xenpf_microcode_update xenpf_microcode_update_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_microcode_update_t);

#define XENPF_platform_quirk      39
#define QUIRK_NOIRQBALANCING      1 /* Do not restrict IO-APIC RTE targets */
#define QUIRK_IOAPIC_BAD_REGSEL   2 /* IO-APIC REGSEL forgets its value    */
#define QUIRK_IOAPIC_GOOD_REGSEL  3 /* IO-APIC REGSEL behaves properly     */
struct xenpf_platform_quirk {
    /* IN variables. */
    uint32_t quirk_id;
};
typedef struct xenpf_platform_quirk xenpf_platform_quirk_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_platform_quirk_t);

#define XENPF_firmware_info       50
#define XEN_FW_DISK_INFO          1 /* from int 13 AH=08/41/48 */
#define XEN_FW_DISK_MBR_SIGNATURE 2 /* from MBR offset 0x1b8 */
#define XEN_FW_VBEDDC_INFO        3 /* from int 10 AX=4f15 */
struct xenpf_firmware_info {
	/* IN variables. */
	uint32_t type;
	uint32_t index;
	/* OUT variables. */
	union {
		struct {
			/* Int13, Fn48: Check Extensions Present. */
			uint8_t device;                   /* %dl: bios device number */
			uint8_t version;                  /* %ah: major version      */
			uint16_t interface_support;       /* %cx: support bitmap     */
			/* Int13, Fn08: Legacy Get Device Parameters. */
			uint16_t legacy_max_cylinder;     /* %cl[7:6]:%ch: max cyl # */
			uint8_t legacy_max_head;          /* %dh: max head #         */
			uint8_t legacy_sectors_per_track; /* %cl[5:0]: max sector #  */
			/* Int13, Fn41: Get Device Parameters (as filled into %ds:%esi). */
			/* NB. First uint16_t of buffer must be set to buffer size.      */
			GUEST_HANDLE(void) edd_params;
		} disk_info; /* XEN_FW_DISK_INFO */
		struct {
			uint8_t device;                   /* bios device number  */
			uint32_t mbr_signature;           /* offset 0x1b8 in mbr */
		} disk_mbr_signature; /* XEN_FW_DISK_MBR_SIGNATURE */
		struct {
			/* Int10, AX=4F15: Get EDID info. */
			uint8_t capabilities;
			uint8_t edid_transfer_time;
			/* must refer to 128-byte buffer */
			GUEST_HANDLE(uchar) edid;
		} vbeddc_info; /* XEN_FW_VBEDDC_INFO */
	} u;
};
typedef struct xenpf_firmware_info xenpf_firmware_info_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_firmware_info_t);

#define XENPF_enter_acpi_sleep    51
struct xenpf_enter_acpi_sleep {
	/* IN variables */
	uint16_t pm1a_cnt_val;      /* PM1a control value. */
	uint16_t pm1b_cnt_val;      /* PM1b control value. */
	uint32_t sleep_state;       /* Which state to enter (Sn). */
	uint32_t flags;             /* Must be zero. */
};
typedef struct xenpf_enter_acpi_sleep xenpf_enter_acpi_sleep_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_enter_acpi_sleep_t);

#define XENPF_change_freq         52
struct xenpf_change_freq {
	/* IN variables */
	uint32_t flags; /* Must be zero. */
	uint32_t cpu;   /* Physical cpu. */
	uint64_t freq;  /* New frequency (Hz). */
};
typedef struct xenpf_change_freq xenpf_change_freq_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_change_freq_t);

/*
 * Get idle times (nanoseconds since boot) for physical CPUs specified in the
 * @cpumap_bitmap with range [0..@cpumap_nr_cpus-1]. The @idletime array is
 * indexed by CPU number; only entries with the corresponding @cpumap_bitmap
 * bit set are written to. On return, @cpumap_bitmap is modified so that any
 * non-existent CPUs are cleared. Such CPUs have their @idletime array entry
 * cleared.
 */
#define XENPF_getidletime         53
struct xenpf_getidletime {
	/* IN/OUT variables */
	/* IN: CPUs to interrogate; OUT: subset of IN which are present */
	GUEST_HANDLE(uchar) cpumap_bitmap;
	/* IN variables */
	/* Size of cpumap bitmap. */
	uint32_t cpumap_nr_cpus;
	/* Must be indexable for every cpu in cpumap_bitmap. */
	GUEST_HANDLE(uint64_t) idletime;
	/* OUT variables */
	/* System time when the idletime snapshots were taken. */
	uint64_t now;
};
typedef struct xenpf_getidletime xenpf_getidletime_t;
DEFINE_GUEST_HANDLE_STRUCT(xenpf_getidletime_t);

struct xen_platform_op {
	uint32_t cmd;
	uint32_t interface_version; /* XENPF_INTERFACE_VERSION */
	union {
		struct xenpf_settime           settime;
		struct xenpf_add_memtype       add_memtype;
		struct xenpf_del_memtype       del_memtype;
		struct xenpf_read_memtype      read_memtype;
		struct xenpf_microcode_update  microcode;
		struct xenpf_platform_quirk    platform_quirk;
		struct xenpf_firmware_info     firmware_info;
		struct xenpf_enter_acpi_sleep  enter_acpi_sleep;
		struct xenpf_change_freq       change_freq;
		struct xenpf_getidletime       getidletime;
		uint8_t                        pad[128];
	} u;
};
typedef struct xen_platform_op xen_platform_op_t;
DEFINE_GUEST_HANDLE_STRUCT(xen_platform_op_t);

#endif /* __XEN_PUBLIC_PLATFORM_H__ */
