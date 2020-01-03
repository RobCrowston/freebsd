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
#include <sys/malloc.h>
#include <sys/smp.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/jail.h>
#include <sys/priv.h>
#include <sys/proc.h>

#include "vmm_host_stat.h"

static MALLOC_DEFINE(M_VMM_HOST_STAT, "vmm host stats",
    "vmm host statistics");

static uint64_t* guest_counters = NULL;
static struct sysctl_ctx_list sysctl_ctx;
static unsigned pr_allow_flag;

static int sysctl_hw_vmm_stat_guest_ticks(SYSCTL_HANDLER_ARGS);

SYSCTL_NODE(_hw_vmm, OID_AUTO, stat, CTLFLAG_RD, 0, "vmm host statistics");

void vmm_host_stat_init()
{
	uint64_t *p;

	p = malloc(sizeof(uint64_t) * mp_ncpus, M_VMM_HOST_STAT,
	    M_WAITOK | M_ZERO);
	atomic_set_rel_ptr((uint64_t *) &guest_counters, (uint64_t) p);

	sysctl_ctx_init(&sysctl_ctx);
	SYSCTL_ADD_PROC(&sysctl_ctx, SYSCTL_STATIC_CHILDREN(_hw_vmm_stat),
	    OID_AUTO, "guest_ticks", CTLTYPE_U64|CTLFLAG_RD|CTLFLAG_MPSAFE, 0,
	    0, sysctl_hw_vmm_stat_guest_ticks, "QU",
	    "Ticks each CPU has spent in guest execution");

	pr_allow_flag = prison_add_allow(NULL, "vmm_host_stat", NULL,
	    "Allow jailed processes to read hw.vmm.stat.");
}

static uint64_t
*get_guest_counters()
{
	uint64_t *p;

	p = (uint64_t *) atomic_load_acq_ptr((uint64_t *) &guest_counters);
	KASSERT(p, "vmm: expecting guest counters to be initialized");
	return p;
}

void vmm_host_stat_cleanup()
{
	uint64_t *p;

	/* Destroy the sysctl context before we free the counters. */
	sysctl_ctx_free(&sysctl_ctx);

	p = get_guest_counters();
	free(p, M_VMM_HOST_STAT);
}

void vmm_host_stat_cpu_ticks_incr(int pcpu, uint64_t val)
{
	uint64_t volatile *p;

	KASSERT(pcpu < mp_ncpus, "Invalid CPU ID for guest tick counter.");

	p = get_guest_counters();
	atomic_add_64((p + pcpu), val);
}

static int
check_sysctl_priv(struct thread *td)
{

	if (jailed(td->td_ucred) && !prison_allow(td->td_ucred, pr_allow_flag))
	    return (EPERM);

	return (priv_check(td, PRIV_VMM_HOST_STAT));
}

static int
sysctl_hw_vmm_stat_guest_ticks(SYSCTL_HANDLER_ARGS)
{
	int error;
	int cpu;
	uint64_t volatile *p;
	uint64_t ticks;

	error = check_sysctl_priv(req->td);
	if (error)
	    return error;

	if (!req->oldptr)
	    return SYSCTL_OUT(req, 0, sizeof(uint64_t) * mp_ncpus);

	p = get_guest_counters();
	for (error = 0, cpu = 0; error == 0 && cpu < mp_ncpus; cpu++) {
		ticks = atomic_load_64(p + cpu);
		error = SYSCTL_OUT(req, &ticks, sizeof(uint64_t));
	}
	return error;
}

