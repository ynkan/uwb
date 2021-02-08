/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM dw3000

#if !defined(__DW3000_TRACE) || defined(TRACE_HEADER_MULTI_READ)
#define __DW3000_TRACE

#include <linux/tracepoint.h>

#include "dw3000.h"
#include "dw3000_testmode_nl.h"

#define MAXNAME 32
#define DW_ENTRY __array(char, dw_name, MAXNAME)
#define DW_ASSIGN strlcpy(__entry->dw_name, dw->dev->kobj.name, MAXNAME)
#define DW_PR_FMT "%s"
#define DW_PR_ARG __entry->dw_name

#ifdef CONFIG_MCPS802154_TESTMODE
#define dw3000_tm_cmd_name(name)            \
	{                                   \
		DW3000_TM_CMD_##name, #name \
	}
#endif

/* We don't want clang-format to modify the following events definition!
   Look at net/wireless/trace.h for the required format. */
/* clang-format off */

/*************************************************************
 *		dw3000 functions return traces		     *
 *************************************************************/

DECLARE_EVENT_CLASS(dw_only_evt,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw),
	TP_STRUCT__entry(
		DW_ENTRY
	),
	TP_fast_assign(
		DW_ASSIGN;
	),
	TP_printk(DW_PR_FMT, DW_PR_ARG)
);

DEFINE_EVENT(dw_only_evt, dw3000_return_void,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_return_int,
	TP_PROTO(struct dw3000 *dw, int ret),
	TP_ARGS(dw, ret),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, ret)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ret = ret;
	),
	TP_printk(DW_PR_FMT ", return %d", DW_PR_ARG, __entry->ret)
);

TRACE_EVENT(dw3000_return_int_u32,
	TP_PROTO(struct dw3000 *dw, int ret, u32 val),
	TP_ARGS(dw, ret, val),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, ret)
		__field(u32, val)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ret = ret;
		__entry->val = val;
	),
	TP_printk(DW_PR_FMT ", return %d, value %u", DW_PR_ARG,
		  __entry->ret, __entry->val)
);

TRACE_EVENT(dw3000_return_int_u64,
	TP_PROTO(struct dw3000 *dw, int ret, u64 val),
	TP_ARGS(dw, ret, val),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, ret)
		__field(u64, val)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ret = ret;
		__entry->val = val;
	),
	TP_printk(DW_PR_FMT ", return %d, value %llu", DW_PR_ARG,
		  __entry->ret, __entry->val)
);

/*************************************************************
 *		dw3000 mcps functions traces		     *
 *************************************************************/

DEFINE_EVENT(dw_only_evt, dw3000_mcps_start,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_stop,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_mcps_tx_frame,
	TP_PROTO(struct dw3000 *dw, u16 len),
	TP_ARGS(dw, len),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u16, len)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->len = len;
	),
	TP_printk(DW_PR_FMT ", skb->len: %d", DW_PR_ARG, __entry->len)
);

TRACE_EVENT(dw3000_mcps_rx_enable,
	TP_PROTO(struct dw3000 *dw, u8 flags, int timeout),
	TP_ARGS(dw, flags, timeout),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, flags)
		__field(int, timeout)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->flags = flags;
		__entry->timeout = timeout;
	),
	TP_printk(DW_PR_FMT ", flags: 0x%x, timeout: %d", DW_PR_ARG,
		  __entry->flags, __entry->timeout)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_rx_disable,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_mcps_rx_get_frame,
	TP_PROTO(struct dw3000 *dw, u16 flags),
	TP_ARGS(dw, flags),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u16, flags)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->flags = flags;
	),
	TP_printk(DW_PR_FMT ", flags: 0x%x", DW_PR_ARG, __entry->flags)
);

TRACE_EVENT(dw3000_mcps_rx_get_error_frame,
	TP_PROTO(struct dw3000 *dw, u16 flags),
	TP_ARGS(dw, flags),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u16, flags)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->flags = flags;
	),
	TP_printk(DW_PR_FMT ", flags: 0x%x", DW_PR_ARG, __entry->flags)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_reset,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_get_timestamp,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_mcps_set_channel,
	TP_PROTO(struct dw3000 *dw, u8 page, u8 channel, u8 pcode),
	TP_ARGS(dw, page, channel, pcode),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, page)
		__field(u8, channel)
		__field(u8, pcode)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->page = page;
		__entry->channel = channel;
		__entry->pcode = pcode;
	),
	TP_printk(DW_PR_FMT ", page: %u, channel: %u, preamble_code: %u",
		  DW_PR_ARG, __entry->page, __entry->channel,
		  __entry->pcode)
);

TRACE_EVENT(dw3000_mcps_set_hw_addr_filt,
	TP_PROTO(struct dw3000 *dw, u8 changed),
	TP_ARGS(dw, changed),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, changed)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->changed = changed;
	),
	TP_printk(DW_PR_FMT ", changed: 0x%x", DW_PR_ARG,
		  __entry->changed)
);

/*************************************************************
 *		dw3000 core functions traces		     *
 *************************************************************/

TRACE_EVENT(dw3000_isr,
	TP_PROTO(struct dw3000 *dw, u32 status),
	TP_ARGS(dw, status),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, status)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->status = status;
	),
	TP_printk(DW_PR_FMT ", status: 0x%x", DW_PR_ARG, __entry->status)
);

DEFINE_EVENT(dw_only_evt, dw3000_read_rx_timestamp,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_power_stats,
	TP_PROTO(struct dw3000 *dw, int state, u64 boot_time, int len_or_date),
	TP_ARGS(dw, state, boot_time, len_or_date),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, state)
		__field(u64, boot_time)
		__field(int, len_or_date)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->state = state;
		__entry->boot_time = boot_time;
		__entry->len_or_date = len_or_date;
	),
	TP_printk(DW_PR_FMT ", state: %d, boot_time: %llu, len_or_date: %u",
		  DW_PR_ARG, __entry->state, __entry->boot_time,
		  (unsigned)__entry->len_or_date)
);

/*************************************************************
 *		dw3000 optional functions traces	     *
 *************************************************************/

#ifdef CONFIG_MCPS802154_TESTMODE
TRACE_EVENT(dw3000_tm_cmd,
	TP_PROTO(struct dw3000 *dw, u32 cmd),
	TP_ARGS(dw, cmd),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, cmd)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->cmd = cmd;
	),
	TP_printk(DW_PR_FMT ", testmode command: %s", DW_PR_ARG,
		__print_symbolic(__entry->cmd,
			dw3000_tm_cmd_name(START_RX_DIAG),
			dw3000_tm_cmd_name(STOP_RX_DIAG),
			dw3000_tm_cmd_name(GET_RX_DIAG),
			dw3000_tm_cmd_name(CLEAR_RX_DIAG),
			dw3000_tm_cmd_name(START_TX_CWTONE),
			dw3000_tm_cmd_name(STOP_TX_CWTONE),
			dw3000_tm_cmd_name(OTP_READ),
			dw3000_tm_cmd_name(OTP_WRITE))
		)
);
#endif

/* clang-format on */
#endif /* !__DW3000_TRACE || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE dw3000_trc
#include <trace/define_trace.h>
