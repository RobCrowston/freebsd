/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2019 Dr Robert Harvey Crowston <crowston@protonmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/smp.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include "vmm_host_stat.h"

static MALLOC_DEFINE(M_VMM_HOST_STAT, "vmm host stats",
    "measurements of the cost on the host of executing guests");

static volatile uint64_t* counters_ptr = NULL;

static uint64_t* get_counters()
{
	uint64_t* old_ptr, * new_ptr;

	old_ptr = (uint64_t *) atomic_load_acq_ptr((uint64_t *) &counters_ptr);
	if (old_ptr)
	    return old_ptr;

	new_ptr = malloc(sizeof(uint64_t) * mp_ncpus, M_VMM_HOST_STAT,
	    M_WAITOK | M_ZERO);

	if (atomic_cmpset_rel_ptr((uint64_t *) &counters_ptr, 0,
	    (uint64_t) new_ptr))
	    return new_ptr;

	/* Competing thread allocated before we did. */
	free(new_ptr, M_VMM_HOST_STAT);

	old_ptr = (uint64_t *) atomic_load_acq_ptr((uint64_t *) &counters_ptr);
	KASSERT(old_ptr, "Not expecting cpu guest tick counter to be null.");
	return old_ptr;
}

void vmm_host_stat_cpu_ticks_incr(int pcpu, uint64_t val)
{
	uint64_t volatile *counters;

	KASSERT(pcpu < mp_ncpus, "Invalid CPU ID for guest tick counter.");

	counters = get_counters();
	atomic_add_64((counters + pcpu), val);
}

static int
sysctl_hw_vmm_stat_guest_ticks(SYSCTL_HANDLER_ARGS)
{
	int error;
	int cpu;
	uint64_t volatile *counters;
	uint64_t ticks;

	if (!req->oldptr)
	    return SYSCTL_OUT(req, 0, sizeof(uint64_t) * mp_ncpus);

	counters = get_counters();
	for (error = 0, cpu = 0; error == 0 && cpu < mp_ncpus; cpu++)
	{
		ticks = atomic_load_64(counters + cpu);
		error = SYSCTL_OUT(req, &ticks, sizeof(uint64_t));
	}
	return error;

}

SYSCTL_DECL(_hw_vmm);
SYSCTL_NODE(_hw_vmm, OID_AUTO, stat, CTLFLAG_RD, 0, "vmm host statistics");
SYSCTL_PROC(_hw_vmm_stat, OID_AUTO, guest_ticks,
    CTLTYPE_U64|CTLFLAG_RD|CTLFLAG_MPSAFE, 0, 0, sysctl_hw_vmm_stat_guest_ticks,
    "LU", "Ticks each CPU has spent in guest execution");

