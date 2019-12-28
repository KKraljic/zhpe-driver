/*
 * Copyright (C) 2017-2018 Hewlett Packard Enterprise Development LP.
 * All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * BSD license below:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ZHPE_OFFLOADED_H_
#define _ZHPE_OFFLOADED_H_

#ifdef __KERNEL__

#include <linux/uio.h>
#include <linux/uuid.h>
#include <linux/socket.h>
#include <asm/byteorder.h>

#define htobe64 cpu_to_be64
#define be64toh be64_to_cpu
#define htobe32 cpu_to_be32
#define be32toh be32_to_cpu

typedef long long       llong;
typedef unsigned long long ullong;

#else

#include <endian.h>
#include <stddef.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <uuid/uuid.h>

#endif

#include "zhpe_offloaded_uapi.h"
#include "debugging_macros.h"

_EXTERN_C_BEG

#define DRIVER_NAME     "zhpe_offloaded"

enum {
    ZHPE_OFFLOADED_OP_INIT,
    ZHPE_OFFLOADED_OP_MR_REG,
    ZHPE_OFFLOADED_OP_MR_FREE,
    ZHPE_OFFLOADED_OP_NOP,
    ZHPE_OFFLOADED_OP_RMR_IMPORT,
    ZHPE_OFFLOADED_OP_RMR_FREE,
    ZHPE_OFFLOADED_OP_ZMMU_REG,
    ZHPE_OFFLOADED_OP_ZMMU_FREE,
    ZHPE_OFFLOADED_OP_UUID_IMPORT,
    ZHPE_OFFLOADED_OP_UUID_FREE,
    ZHPE_OFFLOADED_OP_XQALLOC,
    ZHPE_OFFLOADED_OP_XQFREE,
    ZHPE_OFFLOADED_OP_RQALLOC,
    ZHPE_OFFLOADED_OP_RQFREE,
    ZHPE_OFFLOADED_OP_RESPONSE = 0x80,
    ZHPE_OFFLOADED_OP_VERSION = 1,
};

enum {
    DEBUG_TESTMODE      = 0x00000001,
    DEBUG_MEM           = 0x00000002,
    DEBUG_COUNT         = 0x00000004,
    DEBUG_IO            = 0x00000008,
    DEBUG_RELEASE       = 0x00000010,
    DEBUG_PCI           = 0x00000020,
    DEBUG_ZMMU          = 0x00000040,
    DEBUG_MEMREG        = 0x00000080,
    DEBUG_XQUEUE        = 0x00000100,
    DEBUG_UUID          = 0x00000200,
    DEBUG_MMAP          = 0x00000400,
    DEBUG_RQUEUE        = 0x00000800,
    DEBUG_RKEYS         = 0x00001000,
    DEBUG_MSG           = 0x00002000,
    DEBUG_INTR          = 0x00004000,
};

/* ZHPE_OFFLOADED_MAGIC == 'GENZO'
 * i.e. gen-z offloaded
 * */
#define ZHPE_OFFLOADED_MAGIC      (0x47454e5a4f)

#define ZHPE_OFFLOADED_ENTRY_LEN  (64U)

struct zhpe_offloaded_info {
    uint32_t            qlen;
    uint32_t            rsize;
    uint32_t            qsize;
    uint64_t            reg_off;
    uint64_t            wq_off;
    uint64_t            cq_off;
};

struct zhpe_offloaded_common_hdr {
    uint8_t             version;
    uint8_t             opcode;
    uint16_t            index;
    int                 status;
};

struct zhpe_offloaded_req_INIT {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_rsp_INIT {
    struct zhpe_offloaded_common_hdr hdr;
    uuid_t              uuid;
    uint64_t            global_shared_offset; /* triggered counters */
    uint32_t            global_shared_size;
    uint64_t            local_shared_offset;  /* handled counters */
    uint32_t            local_shared_size;
};

struct zhpe_offloaded_req_MR_REG {
    struct zhpe_offloaded_common_hdr hdr;
    uint64_t               vaddr;
    uint64_t               len;
    uint64_t               access;
};

struct zhpe_offloaded_rsp_MR_REG {
    struct zhpe_offloaded_common_hdr hdr;
    uint64_t               rsp_zaddr;
    uint32_t               pg_ps;
    uint64_t               physaddr;  /* Revisit: remove when IOMMU works */
};

struct zhpe_offloaded_req_MR_FREE {
    struct zhpe_offloaded_common_hdr hdr;
    uint64_t               vaddr;
    uint64_t               len;
    uint64_t               access;
    uint64_t               rsp_zaddr;
};

struct zhpe_offloaded_rsp_MR_FREE {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_req_RMR_IMPORT {
    struct zhpe_offloaded_common_hdr hdr;
    uuid_t                 uuid;
    uint64_t               rsp_zaddr;
    uint64_t               len;
    uint64_t               access;
};

struct zhpe_offloaded_rsp_RMR_IMPORT {
    struct zhpe_offloaded_common_hdr hdr;
    uint64_t               req_addr;
    off_t                  offset;  /* if cpu-visible */
    uint32_t               pg_ps;
};

struct zhpe_offloaded_req_RMR_FREE {
    struct zhpe_offloaded_common_hdr hdr;
    uuid_t                 uuid;
    uint64_t               rsp_zaddr;
    uint64_t               len;
    uint64_t               access;
    uint64_t               req_addr;
};

struct zhpe_offloaded_rsp_RMR_FREE {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_req_NOP {
    struct zhpe_offloaded_common_hdr hdr;
    uint64_t            seq;
};

struct zhpe_offloaded_rsp_NOP {
    struct zhpe_offloaded_common_hdr hdr;
    uint64_t            seq;
};

struct zhpe_offloaded_req_ZMMU_REG {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_rsp_ZMMU_REG {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_req_ZMMU_FREE {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_rsp_ZMMU_FREE {
    struct zhpe_offloaded_common_hdr hdr;
};

enum {
    UUID_IS_FAM = 0x1,
};

struct zhpe_offloaded_req_UUID_IMPORT {
    struct zhpe_offloaded_common_hdr  hdr;
    uuid_t                  uuid;
    uuid_t                  mgr_uuid;
    uint32_t                uu_flags;
};

struct zhpe_offloaded_rsp_UUID_IMPORT {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_req_UUID_FREE {
    struct zhpe_offloaded_common_hdr hdr;
    uuid_t                 uuid;
};

struct zhpe_offloaded_rsp_UUID_FREE {
    struct zhpe_offloaded_common_hdr hdr;
};

/* Defines for the XQALLOC/RQALLOC slice_mask */
#define SLICE_DEMAND 0x80
#define ALL_SLICES 0x0f

struct zhpe_offloaded_qcm {
    uint32_t           size;   /* Bytes allocated for the QCM */
    uint64_t           off;    /* File descriptor offset to the QCM */
};

struct zhpe_offloaded_queue {
    uint32_t           ent;    /* Number of entries in the queue */
    uint32_t           size;   /* Bytes allocated for the queue */
    uint64_t           off;    /* File descriptor offset to the queue */
};

struct zhpe_offloaded_xqinfo {
    struct zhpe_offloaded_qcm     qcm;   /* XDM Queue Control Memory */
    struct zhpe_offloaded_queue   cmdq;  /* XDM Command Queue */
    struct zhpe_offloaded_queue   cmplq; /* XDM Completion Queue */
    uint8_t             slice; /* HW slice number which allocated the queues */
    uint8_t             queue; /* HW queue number */
};

/*
 * Traffic class abstraction for user space. Used in zhpe_offloaded_req_XQALLOC
 * traffic_class field. Mapping to actual Gen-Z traffic class is
 * undefined to user space.
 */
enum {
	ZHPE_OFFLOADED_TC_0 = 0,
	ZHPE_OFFLOADED_TC_1 = 1,
	ZHPE_OFFLOADED_TC_2 = 2,
	ZHPE_OFFLOADED_TC_3 = 3,
	ZHPE_OFFLOADED_TC_4 = 4,
	ZHPE_OFFLOADED_TC_5 = 5,
	ZHPE_OFFLOADED_TC_6 = 6,
	ZHPE_OFFLOADED_TC_7 = 7,
	ZHPE_OFFLOADED_TC_8 = 8,
	ZHPE_OFFLOADED_TC_9 = 9,
	ZHPE_OFFLOADED_TC_10 = 10,
	ZHPE_OFFLOADED_TC_11 = 11,
	ZHPE_OFFLOADED_TC_12 = 12,
	ZHPE_OFFLOADED_TC_13 = 13,
	ZHPE_OFFLOADED_TC_14 = 14,
	ZHPE_OFFLOADED_TC_15 = 15
};

struct zhpe_offloaded_req_XQALLOC {
     struct zhpe_offloaded_common_hdr hdr;
     uint32_t            cmdq_ent;           /* Minimum entries in the cmdq */
     uint32_t            cmplq_ent;          /* Minimum entries in the cmplq */
     uint8_t             traffic_class;      /* Traffic class for this queue */
     uint8_t             priority;           /* Priority for this queue */
     uint8_t             slice_mask;         /* Control HW slice allocation */
};

struct zhpe_offloaded_rsp_XQALLOC {
    struct zhpe_offloaded_common_hdr	hdr;
    struct zhpe_offloaded_xqinfo   	info;
};

struct zhpe_offloaded_req_XQFREE {
    struct zhpe_offloaded_common_hdr	 hdr;
    struct zhpe_offloaded_xqinfo		 info;
};

struct zhpe_offloaded_rsp_XQFREE {
    struct zhpe_offloaded_common_hdr hdr;
};

struct zhpe_offloaded_rqinfo {
    struct zhpe_offloaded_qcm     qcm;   /* XDM Queue Control Memory */
    struct zhpe_offloaded_queue   cmplq; /* XDM Completion Queue */
    uint8_t             slice; /* HW slice number which allocated the queues */
    uint8_t             queue; /* HW queue number */
    uint32_t            rspctxid; /* RSPCTXID to use with EnqA */
    uint32_t            irq_vector; /* interrupt vector that maps to poll dev */
};

struct zhpe_offloaded_req_RQALLOC {
     struct zhpe_offloaded_common_hdr hdr;
     uint32_t            cmplq_ent;          /* Entries in the cmplq minus 1.
					      * e.g. use 1 for 2 entries.  */
     uint8_t             slice_mask;         /* Control HW slice allocation */
};

struct zhpe_offloaded_rsp_RQALLOC {
     struct zhpe_offloaded_common_hdr     hdr;
     struct zhpe_offloaded_rqinfo   	info;
};

struct zhpe_offloaded_req_RQFREE {
     struct zhpe_offloaded_common_hdr     hdr;
     struct zhpe_offloaded_rqinfo   	info;
};

struct zhpe_offloaded_rsp_RQFREE {
     struct zhpe_offloaded_common_hdr hdr;
};

union zhpe_offloaded_req {
    struct zhpe_offloaded_common_hdr hdr;
    struct zhpe_offloaded_req_INIT        init;
    struct zhpe_offloaded_req_MR_REG      mr_reg;
    struct zhpe_offloaded_req_MR_FREE     mr_free;
    struct zhpe_offloaded_req_RMR_IMPORT  rmr_import;
    struct zhpe_offloaded_req_RMR_FREE    rmr_free;
    struct zhpe_offloaded_req_NOP         nop;
    struct zhpe_offloaded_req_ZMMU_REG    zmmu_reg;
    struct zhpe_offloaded_req_ZMMU_FREE   zmmu_free;
    struct zhpe_offloaded_req_UUID_IMPORT uuid_import;
    struct zhpe_offloaded_req_UUID_FREE   uuid_free;
    struct zhpe_offloaded_req_XQALLOC     xqalloc;
    struct zhpe_offloaded_req_XQFREE      xqfree;
    struct zhpe_offloaded_req_RQALLOC     rqalloc;
    struct zhpe_offloaded_req_RQFREE      rqfree;
};

union zhpe_offloaded_rsp {
    struct zhpe_offloaded_common_hdr hdr;
    struct zhpe_offloaded_rsp_INIT        init;
    struct zhpe_offloaded_rsp_MR_REG      mr_reg;
    struct zhpe_offloaded_rsp_MR_FREE     mr_free;
    struct zhpe_offloaded_rsp_RMR_IMPORT  rmr_import;
    struct zhpe_offloaded_rsp_RMR_FREE    rmr_free;
    struct zhpe_offloaded_rsp_NOP         nop;
    struct zhpe_offloaded_rsp_ZMMU_REG    zmmu_reg;
    struct zhpe_offloaded_rsp_ZMMU_FREE   zmmu_free;
    struct zhpe_offloaded_rsp_UUID_IMPORT uuid_import;
    struct zhpe_offloaded_rsp_UUID_FREE   uuid_free;
    struct zhpe_offloaded_rsp_XQALLOC     xqalloc;
    struct zhpe_offloaded_rsp_XQFREE      xqfree;
    struct zhpe_offloaded_rsp_RQALLOC     rqalloc;
    struct zhpe_offloaded_rsp_RQFREE      rqfree;
};

union zhpe_offloaded_op {
    struct zhpe_offloaded_common_hdr hdr;
    union zhpe_offloaded_req     req;
    union zhpe_offloaded_rsp     rsp;
};

#define ZHPE_OFFLOADED_GLOBAL_SHARED_VERSION    (1)
#define SLICES                        4
#define VECTORS_PER_SLICE             32
#define MAX_IRQ_VECTORS               (VECTORS_PER_SLICE * SLICES)

struct zhpe_offloaded_global_shared_data {
    uint                magic;
    uint                version;
    uint                debug_flags;
    struct zhpe_offloaded_attr    default_attr;
    uint32_t            triggered_counter[MAX_IRQ_VECTORS];
};

#define ZHPE_OFFLOADED_LOCAL_SHARED_VERSION    (1)
struct zhpe_offloaded_local_shared_data {
    uint                magic;
    uint                version;
    uint32_t            handled_counter[MAX_IRQ_VECTORS];
};

/* XDM QCM access macros and structures. Reads and writes must be 64 bits */

struct zhpe_offloaded_xdm_active_status_error {
    uint64_t active_cmd_cnt   : 11;
    uint64_t rv1              : 4;
    uint64_t active           : 1;
    uint64_t status           : 3;
    uint64_t rv2              : 12;
    uint64_t error            : 1;
    uint64_t rv3              : 32;
};
#define ZHPE_OFFLOADED_XDM_QCM_ACTIVE_STATUS_ERROR_OFFSET	0x28
#define ZHPE_OFFLOADED_XDM_QCM_STOP_OFFSET		0x40
#define ZHPE_OFFLOADED_XDM_QCM_CMD_QUEUE_TAIL_OFFSET	0x80
#define ZHPE_OFFLOADED_XDM_QCM_CMD_QUEUE_HEAD_OFFSET	0xc0
struct zhpe_offloaded_xdm_cmpl_queue_tail_toggle {
    uint64_t cmpl_q_tail_idx  : 16;
    uint64_t rv1              : 15;
    uint64_t toggle_valid     : 1;
    uint64_t rv2              : 32;
};
#define ZHPE_OFFLOADED_XDM_QCM_CMPL_QUEUE_TAIL_TOGGLE_OFFSET	0x100

/* RDM QCM access macros and structures. Reads and writes must be 64 bits */
#define ZHPE_OFFLOADED_RDM_QCM_ACTIVE				0x18
#define ZHPE_OFFLOADED_RDM_QCM_STOP_OFFSET			0x40
struct zhpe_offloaded_rdm_rcv_queue_tail_toggle {
    uint64_t rcv_q_tail_idx   : 20;
    uint64_t rv1              : 11;
    uint64_t toggle_valid     : 1;
    uint64_t rv2              : 32;
};
#define ZHPE_OFFLOADED_RDM_QCM_RCV_QUEUE_TAIL_TOGGLE_OFFSET	0x80
#define ZHPE_OFFLOADED_RDM_QCM_RCV_QUEUE_HEAD_OFFSET		0xc0

_EXTERN_C_END

#endif /* _ZHPE_OFFLOADED_H_ */
