/**
 *   @ingroup hal
 *   @file
 *
 *   Real-Time Hardware Abstraction Layer for x86.
 *
 *   Original RTAI/x86 HAL services from: \n
 *   Copyright &copy; 2000 Paolo Mantegazza, \n
 *   Copyright &copy; 2000 Steve Papacharalambous, \n
 *   Copyright &copy; 2000 Stuart Hughes, \n
 *   and others.
 *
 *   RTAI/x86 rewrite over Adeos: \n
 *   Copyright &copy; 2002,2003 Philippe Gerum.
 *   Major refactoring for Xenomai: \n
 *   Copyright &copy; 2004,2005 Philippe Gerum.
 *   Arithmetic/conversion routines: \n
 *   Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Xenomai; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 */

#ifndef _XENO_ASM_X86_HAL_32_H
#define _XENO_ASM_X86_HAL_32_H

/*#include <asm/xenomai/wrappers.h>*/ #include <asm-x86/wrappers.h>

#define RTHAL_ARCH_NAME			"i386"
#ifdef CONFIG_IPIPE_CORE
# define RTHAL_TIMER_DEVICE		(ipipe_timer_name())
#elif defined(CONFIG_X86_LOCAL_APIC)
# define RTHAL_TIMER_DEVICE		"lapic"
#else
# define RTHAL_TIMER_DEVICE		"pit"
#endif
#if IPIPE_CORE_APIREV >= 2
# define RTHAL_CLOCK_DEVICE		(ipipe_clock_name())
#elif defined(CONFIG_X86_TSC)
# define RTHAL_CLOCK_DEVICE		"tsc"
#else
# define RTHAL_CLOCK_DEVICE		"pit"
#endif

/*#include <asm-generic/xenomai/hal.h>*/ #include <asm-generic/hal.h>	/* Read the generic bits. */

#if defined(CONFIG_X86_LOCAL_APIC) && \
  defined(apic_write_around) && !defined(CONFIG_X86_GOOD_APIC)
#error "Xenomai needs a working LAPIC - if your machine has a bad one, you"
#error "should disable CONFIG_X86_LOCAL_APIC in your kernel configuration."
#endif

typedef unsigned long long rthal_time_t;

static inline __attribute_const__ unsigned long ffnz(unsigned long ul)
{
	/* Derived from bitops.h's ffs() */
      __asm__("bsfl %1, %0":"=r,r"(ul)
      :	"r,?m"(ul));
	return ul;
}

#ifndef __cplusplus
#include <asm/io.h>
#include <asm/timex.h>
#include <asm/processor.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
#include <asm/i8259.h>
#else
#include <io_ports.h>
#endif
#ifdef CONFIG_X86_LOCAL_APIC
#include <asm/fixmap.h>
#include <asm/apic.h>
#endif /* CONFIG_X86_LOCAL_APIC */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#include <linux/i8253.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
#include <asm/i8253.h>
#endif
#include <asm/msr.h>
/*#include <asm/xenomai/atomic.h>*/ #include <asm-x86/atomic.h>
/*#include <asm/xenomai/smi.h>*/ #include <asm-x86/smi.h>
/*#include <asm/xenomai/c1e.h>*/ #include <asm-x86/c1e.h>

#ifdef CONFIG_IPIPE_CORE
#define RTHAL_TIMER_IRQ		__ipipe_hrtimer_irq
#define RTHAL_HOST_TICK_IRQ	__ipipe_hrtimer_irq
#define RTHAL_TIMER_IPI		RTHAL_HRTIMER_IPI
#elif defined(CONFIG_X86_LOCAL_APIC)
#define RTHAL_HRTIMER_VECTOR	IPIPE_SERVICE_VECTOR0
#define RTHAL_APIC_TIMER_VECTOR	RTHAL_HRTIMER_VECTOR
#define RTHAL_TIMER_IPI	RTHAL_HRTIMER_IPI
#define RTHAL_APIC_ICOUNT	((RTHAL_TIMER_FREQ + HZ/2)/HZ)
#define RTHAL_TIMER_IRQ		RTHAL_TIMER_IPI
#define RTHAL_HOST_TICK_IRQ	ipipe_apic_vector_irq(LOCAL_TIMER_VECTOR)
#define RTHAL_BCAST_TICK_IRQ	0	/* Tick broadcasting interrupt. */
#ifndef ipipe_apic_vector_irq
/* Older I-pipe versions do not differentiate the normal IRQ space
   from the system IRQ range, which is wrong... */
#define ipipe_apic_vector_irq(vec) (vec - FIRST_EXTERNAL_VECTOR)
#endif

#else /* !CONFIG_X86_LOCAL_APIC */

#define RTHAL_TIMER_IRQ		0	/* i8253 PIT interrupt. */
#define RTHAL_HOST_TICK_IRQ	0	/* Host tick is emulated by Xenomai. */

#endif /* CONFIG_X86_LOCAL_APIC */

#define RTHAL_NMICLK_FREQ	RTHAL_CPU_FREQ

static inline void rthal_grab_control(void)
{
	rthal_c1e_disable();
	rthal_smi_init();
	rthal_smi_disable();
}

static inline void rthal_release_control(void)
{
	rthal_smi_restore();
}

#if defined(CONFIG_X86_TSC) || IPIPE_CORE_APIREV >= 2
static inline unsigned long long rthal_rdtsc(void)
{
	unsigned long long t;
	rthal_read_tsc(t);
	return t;
}
#else /* !CONFIG_X86_TSC && IPIPE_CORE_APIREV < 2 */
#define RTHAL_8254_COUNT2LATCH  0xfffe
void rthal_setup_8254_tsc(void);
rthal_time_t rthal_get_8254_tsc(void);
#define rthal_rdtsc() rthal_get_8254_tsc()
#endif /* !CONFIG_X86_TSC && IPIPE_CORE_APIREV < 2 */

static inline void rthal_timer_program_shot(unsigned long delay)
{
/* With head-optimization, callers are expected to have switched off
   hard-IRQs already -- no need for additional protection in this case. */
#ifdef CONFIG_IPIPE_CORE
	ipipe_timer_set(delay);
#else /* !I-pipe core */
#ifndef CONFIG_XENO_OPT_PIPELINE_HEAD
	unsigned long flags;

	rthal_local_irq_save_hw(flags);
#endif /* CONFIG_XENO_OPT_PIPELINE_HEAD */
#ifdef CONFIG_X86_LOCAL_APIC
	if (!delay) {
		/* Pend the timer interrupt. */
		rthal_schedule_irq_head(RTHAL_TIMER_IPI);
	} else {
		/* Note: reading before writing just to work around the Pentium
		   APIC double write bug. apic_read() expands to nil
		   whenever CONFIG_X86_GOOD_APIC is set. --rpm */
		apic_read(APIC_TMICT);
		apic_write(APIC_TMICT, delay);
	}
#else /* !CONFIG_X86_LOCAL_APIC */
	if (!delay)
		rthal_schedule_irq_head(RTHAL_TIMER_IRQ);
	else {
		outb(delay & 0xff, PIT_CH0);
		outb(delay >> 8, PIT_CH0);
	}
#endif /* CONFIG_X86_LOCAL_APIC */
#ifndef CONFIG_XENO_OPT_PIPELINE_HEAD
	rthal_local_irq_restore_hw(flags);
#endif /* CONFIG_XENO_OPT_PIPELINE_HEAD */
#endif /* !I-pipe core */
}

static const char *const rthal_fault_labels[] = {
	[0] = "Divide error",
	[1] = "Debug",
	[2] = "",		/* NMI is not pipelined. */
	[3] = "Int3",
	[4] = "Overflow",
	[5] = "Bounds",
	[6] = "Invalid opcode",
	[7] = "FPU not available",
	[8] = "Double fault",
	[9] = "FPU segment overrun",
	[10] = "Invalid TSS",
	[11] = "Segment not present",
	[12] = "Stack segment",
	[13] = "General protection",
	[14] = "Page fault",
	[15] = "Spurious interrupt",
	[16] = "FPU error",
	[17] = "Alignment check",
	[18] = "Machine check",
	[19] = "SIMD error",
	[20] = NULL,
};

#ifdef CONFIG_X86_LOCAL_APIC

#include <asm/fixmap.h>
#include <asm/mpspec.h>
#ifdef CONFIG_X86_IO_APIC
#include <asm/io_apic.h>
#endif /* CONFIG_X86_IO_APIC */
#include <asm/apic.h>

static inline int rthal_set_apic_base(int lvtt_value)
{
	if (!APIC_INTEGRATED(GET_APIC_VERSION(apic_read(APIC_LVR))))
		lvtt_value |= SET_APIC_TIMER_BASE(APIC_TIMER_BASE_DIV);

	return lvtt_value;
}

static inline void rthal_setup_periodic_apic(int count, int vector)
{
	apic_read(APIC_LVTT);
	apic_write(APIC_LVTT, rthal_set_apic_base(APIC_LVT_TIMER_PERIODIC | vector));
	apic_read(APIC_TMICT);
	apic_write(APIC_TMICT, count);
}

static inline void rthal_setup_oneshot_apic(int vector)
{
	apic_read(APIC_LVTT);
	apic_write(APIC_LVTT, rthal_set_apic_base(vector));
}
#endif /* !CONFIG_X86_LOCAL_APIC */

#endif /* !__cplusplus */

long rthal_strncpy_from_user(char *dst, const char __user * src, long count);

#endif /* !_XENO_ASM_X86_HAL_32_H */
