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
 *
 * MCPS interface, tools to handle frames from a region handler.
 *
 */

#ifndef NET_MCPS802154_FRAME_H
#define NET_MCPS802154_FRAME_H

#include <linux/skbuff.h>

#define IEEE802154_FC_IE_PRESENT_SHIFT 9
#define IEEE802154_FC_IE_PRESENT (1 << IEEE802154_FC_IE_PRESENT_SHIFT)

#define IEEE802154_IE_HEADER_LEN 2
#define IEEE802154_IE_HEADER_TYPE_SHIFT 15
#define IEEE802154_IE_HEADER_TYPE (1 << IEEE802154_IE_HEADER_TYPE_SHIFT)

#define IEEE802154_HEADER_IE_HEADER_LENGTH 0x7f
#define IEEE802154_HEADER_IE_HEADER_ELEMENT_ID (0xff << 7)
#define IEEE802154_HEADER_IE_HEADER_TYPE (0 << IEEE802154_IE_HEADER_TYPE_SHIFT)

#define IEEE802154_PAYLOAD_IE_HEADER_LENGTH 0x7ff
#define IEEE802154_PAYLOAD_IE_HEADER_GROUP_ID (0xf << 11)
#define IEEE802154_PAYLOAD_IE_HEADER_TYPE (1 << IEEE802154_IE_HEADER_TYPE_SHIFT)

#define IEEE802154_LONG_NESTED_IE_HEADER_LENGTH 0x7ff
#define IEEE802154_LONG_NESTED_IE_HEADER_SUB_ID (0xf << 11)
#define IEEE802154_LONG_NESTED_IE_HEADER_TYPE \
	(1 << IEEE802154_IE_HEADER_TYPE_SHIFT)

#define IEEE802154_SHORT_NESTED_IE_HEADER_LENGTH 0xff
#define IEEE802154_SHORT_NESTED_IE_HEADER_SUB_ID (0x7f << 8)
#define IEEE802154_SHORT_NESTED_IE_HEADER_TYPE \
	(0 << IEEE802154_IE_HEADER_TYPE_SHIFT)

#define IEEE802154_IE_NESTED_SHORT_MIN_SID 0x10

#define IEEE802154_IE_HEADER_TERMINATION_1_ID 0x7e
#define IEEE802154_IE_HEADER_TERMINATION_2_ID 0x7f

#define IEEE802154_IE_PAYLOAD_MLME_GID 0x1
#define IEEE802154_IE_PAYLOAD_TERMINATION_GID 0xf

struct mcps802154_llhw;

/**
 * enum mcps802154_ie_get_kind - Kind of IE, or none.
 * @MCPS802154_IE_GET_KIND_NONE: No IE decoded (at end of frame).
 * @MCPS802154_IE_GET_KIND_HEADER: Header IE decoded.
 * @MCPS802154_IE_GET_KIND_PAYLOAD: Payload IE decoded.
 * @MCPS802154_IE_GET_KIND_MLME_NESTED:
 *	Nested IE inside a MLME payload IE decoded.
 */
enum mcps802154_ie_get_kind {
	MCPS802154_IE_GET_KIND_NONE,
	MCPS802154_IE_GET_KIND_HEADER,
	MCPS802154_IE_GET_KIND_PAYLOAD,
	MCPS802154_IE_GET_KIND_MLME_NESTED,
};

/**
 * struct mcps802154_ie_get_context - Context for IE decoding, to be used with
 * mcps802154_ie_get(). Initialize to zero.
 */
struct mcps802154_ie_get_context {
	/**
	 * @kind: Kind of decoded IE.
	 */
	enum mcps802154_ie_get_kind kind;
	/**
	 * @id: Element identifier, group identifier or sub identifier of the
	 * decoded IE.
	 */
	int id;
	/**
	 * @len: Length of the decoded IE.
	 */
	unsigned int len;
	/**
	 * @mlme_len: While an MLME IE is decoded, length of data still in the
	 * frame buffer for this IE. Set this to 0 if you pulled all the MLME
	 * payload.
	 */
	unsigned int mlme_len;
};

/**
 * mcps802154_frame_alloc() - Allocate a buffer for TX.
 * @llhw: Low-level device pointer.
 * @size: Header and payload size.
 * @flags: Allocation mask.
 *
 * This is to allocate a buffer for sending a frame to the low-level driver
 * directly.  Additional space is reserved for low-level driver headroom and for
 * checksum.
 *
 * Return: Allocated buffer, or NULL.
 */
struct sk_buff *mcps802154_frame_alloc(struct mcps802154_llhw *llhw,
				       unsigned int size, gfp_t flags);

/**
 * mcps802154_ie_put_begin() - Begin writing information elements.
 * @skb: Frame buffer.
 *
 * Prepare a frame buffer for writing IEs. The buffer control buffer is used to
 * store state information.
 */
void mcps802154_ie_put_begin(struct sk_buff *skb);

/**
 * mcps802154_ie_put_end() - End writing information elements.
 * @skb: Frame buffer.
 * @data_payload: True if data will be appended to the buffer after the IEs. In
 * this case, a terminator IE may be needed.
 *
 * This function appends a terminator IE if needed.
 *
 * Return: 0 or -ENOBUFS in case of error.
 */
int mcps802154_ie_put_end(struct sk_buff *skb, bool data_payload);

/**
 * mcps802154_ie_put_header_ie() - Add a header IE.
 * @skb: Frame buffer.
 * @element_id: Header IE element identifier.
 * @len: Header IE payload length.
 *
 * This adds the IE header and reserves room to write your payload. This works
 * like skb_put, you must write at the returned address.
 *
 * Return: Address of reserved space to write payload, or NULL in case of error.
 */
void *mcps802154_ie_put_header_ie(struct sk_buff *skb, int element_id,
				  unsigned int len);

/**
 * mcps802154_ie_put_payload_ie() - Add a payload IE.
 * @skb: Frame buffer.
 * @group_id: Payload IE group identifier.
 * @len: Payload IE payload length.
 *
 * This adds the IE header and reserves room to write your payload. This works
 * like skb_put, you must write at the returned address.
 *
 * Return: Address of reserved space to write payload, or NULL in case of error.
 */
void *mcps802154_ie_put_payload_ie(struct sk_buff *skb, int group_id,
				   unsigned int len);

/**
 * mcps802154_ie_put_nested_mlme_ie() - Add a nested IE, inside a MLME IE.
 * @skb: Frame buffer.
 * @sub_id: Nested IE element identifier.
 * @len: Nested IE payload length.
 *
 * This adds the IE header and reserves room to write your payload. This works
 * like skb_put, you must write at the returned address.
 *
 * The MLME payload IE is added automatically if needed and its length is
 * incremented if present yet.
 *
 * Return: Address of reserved space to write payload, or NULL in case of error.
 */
void *mcps802154_ie_put_nested_mlme_ie(struct sk_buff *skb, int sub_id,
				       unsigned int len);

/**
 * mcps802154_ie_get() - Parse one IE and fill context.
 * @skb: Frame buffer.
 * @context: Parse context, should be zero initialized at first call.
 *
 * This should only be called if the buffer contains IEs. This can be determined
 * using the IE_PRESENT bit in the frame control.
 *
 * On successful parsing, the context structure is filled with information about
 * the read IE. The IE payload can be read at the head of the frame buffer,
 * headers are consumed.
 *
 * On last return, 1 is returned and a termination IE can be present in the
 * context, it usually can be ignored.
 *
 * When an MLME IE is found, you have two options:
 *  - ignore it and call again to parse nested IE.
 *  - pull nested payload from the frame buffer, in this case you should set
 *    mlme_len to zero to proceed with the next IE.
 *
 * Return: 1 if last IE, 0 on successfully decoded IE, else negative error code.
 */
int mcps802154_ie_get(struct sk_buff *skb,
		      struct mcps802154_ie_get_context *context);

/**
 * mcps802154_get_extended_addr() - Get current extended address.
 * @llhw: Low-level device pointer.
 *
 * Return: Extended address.
 */
__le64 mcps802154_get_extended_addr(struct mcps802154_llhw *llhw);

/**
 * mcps802154_get_pan_id() - Get current PAN identifier.
 * @llhw: Low-level device pointer.
 *
 * Return: PAN ID.
 */
__le16 mcps802154_get_pan_id(struct mcps802154_llhw *llhw);

/**
 * mcps802154_get_short_addr() - Get current short address.
 * @llhw: Low-level device pointer.
 *
 * Return: Short address.
 */
__le16 mcps802154_get_short_addr(struct mcps802154_llhw *llhw);

/**
 * mcps802154_timestamp_dtu_to_rctu() - Convert a timestamp in device time unit
 * to a timestamp in ranging counter time unit.
 * @llhw: Low-level device pointer.
 * @timestamp_dtu: Timestamp value in device time unit.
 *
 * Return: Timestamp value in ranging counter time unit.
 */
u64 mcps802154_timestamp_dtu_to_rctu(struct mcps802154_llhw *llhw,
				     u32 timestamp_dtu);

/**
 * mcps802154_timestamp_rctu_to_dtu() - Convert a timestamp in ranging counter
 * time unit to a timestamp in device time unit.
 * @llhw: Low-level device pointer.
 * @timestamp_rctu: Timestamp value in ranging counter time unit.
 *
 * Return: Timestamp value in device time unit.
 */
u32 mcps802154_timestamp_rctu_to_dtu(struct mcps802154_llhw *llhw,
				     u64 timestamp_rctu);

/**
 * mcps802154_align_tx_timestamp_rctu() - Align a transmission timestamp so that
 * the transmission can be done at the exact timestamp value (RDEV only).
 * @llhw: Low-level device pointer.
 * @timestamp_rctu: Timestamp value to align.
 *
 * Return: Aligned timestamp.
 */
u64 mcps802154_align_tx_timestamp_rctu(struct mcps802154_llhw *llhw,
				       u64 timestamp_rctu);

/**
 * mcps802154_difference_timestamp_rctu() - Compute the difference between two
 * timestamp values.
 * @llhw: Low-level device pointer.
 * @timestamp_a_rctu: Timestamp A value.
 * @timestamp_b_rctu: Timestamp B value.
 *
 * Return: Difference between A and B.
 */
s64 mcps802154_difference_timestamp_rctu(struct mcps802154_llhw *llhw,
					 u64 timestamp_a_rctu,
					 u64 timestamp_b_rctu);

#endif /* NET_MCPS802154_FRAME_H */
