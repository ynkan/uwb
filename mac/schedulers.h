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
 * 802.15.4 mac common part sublayer, definitions to handle schedulers.
 *
 */

#ifndef NET_MCPS802154_SCHEDULERS_H
#define NET_MCPS802154_SCHEDULERS_H

#include <net/netlink.h>

struct mcps802154_local;
struct mcps802154_scheduler;

/**
 * mcps802154_scheduler_open() - Open a scheduler, and set parameters.
 * @local: MCPS private data.
 * @name: Name of scheduler to open.
 * @params_attr: Nested attribute containing scheduler parameters, may be NULL.
 * @extack: Extended ACK report structure.
 *
 * Return: The open scheduler or NULL on error.
 */
struct mcps802154_scheduler *
mcps802154_scheduler_open(struct mcps802154_local *local, const char *name,
			  const struct nlattr *params_attr,
			  struct netlink_ext_ack *extack);

/**
 * mcps802154_scheduler_close() - Close a scheduler.
 * @scheduler: Pointer to the scheduler.
 */
void mcps802154_scheduler_close(struct mcps802154_scheduler *scheduler);

/**
 * mcps802154_scheduler_set_parameters() - Set parameters of an open scheduler.
 * @scheduler: Pointer to the scheduler.
 * @params_attr: Nested attribute containing scheduler parameters, may be NULL.
 * @extack: Extended ACK report structure.
 *
 * Return: 0 or error.
 */
int mcps802154_scheduler_set_parameters(struct mcps802154_scheduler *scheduler,
					const struct nlattr *params_attr,
					struct netlink_ext_ack *extack);

/**
 * mcps802154_scheduler_list_region_ids() - List the scheduler regions ids.
 * @scheduler: Pointer to the scheduler.
 *
 * Return: NULL ended string array.
 */
const char *const *
mcps802154_scheduler_list_region_ids(struct mcps802154_scheduler *scheduler);

/**
 * mcps802154_scheduler_set_region_parameters() - Set parameters of a specific
 * region in a specific scheduler.
 * @scheduler: Pointer to the scheduler.
 * @region_id: Identifier of the region, scheduler specific, can be NULL.
 * @region_name: Name of region to attach to the scheduler.
 * @params_attr: Nested attribute containing region parameters.
 * @extack: Extended ACK report structure.
 *
 * Return: 0 or error.
 */
int mcps802154_scheduler_set_region_parameters(
	struct mcps802154_scheduler *scheduler, const char *region_id,
	const char *region_name, const struct nlattr *params_attr,
	struct netlink_ext_ack *extack);

/**
 * mcps802154_scheduler_call() - Call scheduler specific procedure.
 * @scheduler: Pointer to the scheduler.
 * @call_id: Identifier of the procedure, scheduler specific.
 * @params_attr: Nested attribute containing procedure parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
int mcps802154_scheduler_call(struct mcps802154_scheduler *scheduler,
			      u32 call_id, const struct nlattr *params_attr,
			      const struct genl_info *info);

/**
 * mcps802154_scheduler_call_region() - Call region specific procedure.
 * @scheduler: Pointer to the scheduler.
 * @region_id: Identifier of the region, scheduler specific, can be NULL.
 * @region_name: Name of the region to call.
 * @call_id: Identifier of the procedure, region specific.
 * @params_attr: Nested attribute containing procedure parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
int mcps802154_scheduler_call_region(struct mcps802154_scheduler *scheduler,
				     const char *region_id,
				     const char *region_name, u32 call_id,
				     const struct nlattr *params_attr,
				     const struct genl_info *info);

#endif /* NET_MCPS802154_SCHEDULERS_H */
