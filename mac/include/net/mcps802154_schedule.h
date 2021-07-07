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
 * MCPS schedule interface.
 */

#ifndef NET_MCPS802154_SCHEDULE_H
#define NET_MCPS802154_SCHEDULE_H

#include <linux/list.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <net/genetlink.h>
#include <net/netlink.h>

#include <net/mcps802154.h>

struct mcps802154_nl_ranging_request;

/**
 * MCPS802154_DURATION_NO_CHANGE - Do not change duration.
 */
#define MCPS802154_DURATION_NO_CHANGE (-1)

/**
 * enum mcps802154_access_method - Method to implement an access.
 * @MCPS802154_ACCESS_METHOD_NOTHING:
 *      Nothing to do, wait for a schedule change.
 * @MCPS802154_ACCESS_METHOD_IMMEDIATE_RX:
 *	RX as soon as possible, without timeout, with auto-ack.
 * @MCPS802154_ACCESS_METHOD_IMMEDIATE_TX:
 *	TX as soon as possible. Could be with or without ack request (AR).
 * @MCPS802154_ACCESS_METHOD_MULTI:
 *	Multiple frames described in frames table.
 * @MCPS802154_ACCESS_METHOD_VENDOR:
 *      Route all signals to access callbacks for vendor specific handling.
 */
enum mcps802154_access_method {
	MCPS802154_ACCESS_METHOD_NOTHING,
	MCPS802154_ACCESS_METHOD_IMMEDIATE_RX,
	MCPS802154_ACCESS_METHOD_IMMEDIATE_TX,
	MCPS802154_ACCESS_METHOD_MULTI,
	MCPS802154_ACCESS_METHOD_VENDOR,
};

/**
 * enum mcps802154_access_tx_return_reason - Reason of TX buffer return.
 * @MCPS802154_ACCESS_TX_RETURN_REASON_CONSUMED:
 *	Frame was sent successfully.
 * @MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE:
 *	An attempt was done to deliver the frame, but it failed.
 * @MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL:
 *	No attempt was done to deliver the frame, or there was an unexpected
 *	error doing it.
 */
enum mcps802154_access_tx_return_reason {
	MCPS802154_ACCESS_TX_RETURN_REASON_CONSUMED,
	MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE,
	MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL,
};

/**
 * struct mcps802154_access_frame - Information for a single frame for multiple
 * frames method.
 */
struct mcps802154_access_frame {
	/**
	 * @is_tx: True if frame is TX, else RX.
	 */
	bool is_tx;
	union {
		/**
		 * @tx_frame_info: Information for transmitting a frame. Should
		 * have rx_enable_after_tx_dtu == 0.
		 */
		struct mcps802154_tx_frame_info tx_frame_info;
		/**
		 * @rx: Information for receiving a frame.
		 */
		struct {
			/**
			 * @rx.info: Information for enabling the receiver.
			 */
			struct mcps802154_rx_info info;
			/**
			 * @rx.frame_info_flags_request: Information to request
			 * when a frame is received, see
			 * &enum mcps802154_rx_frame_info_flags.
			 */
			u16 frame_info_flags_request;
		} rx;
	};
	/**
	 * @sts_params: Pointer to STS parameters for this frame and all
	 * following frames. STS is still only used if requested in flags. For
	 * TX, this is read after mcps802154_access::tx_get_frame() is called,
	 * so it can be changed by the callback. For RX, this is read earlier,
	 * so it needs to be valid after mcps802154_access_ops::get_access(), or
	 * after the previous callback.
	 */
	const struct mcps802154_sts_params *sts_params;
};

/**
 * struct mcps802154_region_demand - Access information for on demand
 * schedulers.
 */
struct mcps802154_region_demand {
	/**
	 * @timestamp_dtu: Start of the demand.
	 */
	u32 timestamp_dtu;
	/**
	 * @duration_dtu: Duration of the demand, 0 for endless.
	 */
	int duration_dtu;
};

/**
 * struct mcps802154_access - Single medium access.
 *
 * This structure gives MCPS all the information needed to perform a single
 * access.
 */
struct mcps802154_access {
	/**
	 * @method: Method of access, see &enum mcps802154_access_method.
	 */
	enum mcps802154_access_method method;
	union {
		/**
		 * @ops: Callbacks to implement the access.
		 */
		struct mcps802154_access_ops *ops;
		/**
		 * @vendor_ops: Callbacks to implement the vendor specific
		 * access.
		 */
		struct mcps802154_access_vendor_ops *vendor_ops;
	};
	/**
	 * @timestamp_dtu: Start of the access, only valid when the access has
	 * a duration. Invalid for immediate accesses.
	 */
	u32 timestamp_dtu;
	/**
	 * @duration_dtu: Duration of the access, or 0 if unknown (this is the
	 * case if the first frame is a RX with no timeout and the frame has not
	 * been received yet).
	 */
	int duration_dtu;
	/**
	 * @n_frames: Number of frames in an access using multiple frames
	 * method. This can be changed by the &mcps802154_access_ops.rx_frame()
	 * callback.
	 */
	size_t n_frames;
	/**
	 * @frames: Table of information for each frames in an access using
	 * multiple frames method. This can be changed by the
	 * &mcps802154_access_ops.rx_frame() callback.
	 */
	struct mcps802154_access_frame *frames;
};

/**
 * struct mcps802154_access_ops - Callbacks to implement an access.
 */
struct mcps802154_access_ops {
	/*
	 * Anonymous structure which must be declared at the beginning of all
	 * access ops.
	 */
	struct {
		/**
		 * @access_done: Called when the access is done, successfully or
		 * not.
		 */
		void (*access_done)(struct mcps802154_access *access);
	};
	/**
	 * @rx_frame: Once a frame is received, it is given to this function.
	 * Buffer ownership is transferred to the callee.
	 *
	 * For multiple frames access method, this is called with NULL pointers
	 * to report RX timeout or error.
	 */
	void (*rx_frame)(struct mcps802154_access *access, int frame_idx,
			 struct sk_buff *skb,
			 const struct mcps802154_rx_frame_info *info);
	/**
	 * @tx_get_frame: Return a frame to send, the buffer is lend to caller
	 * and should be returned with &mcps802154_access_ops.tx_return().
	 */
	struct sk_buff *(*tx_get_frame)(struct mcps802154_access *access,
					int frame_idx);
	/**
	 * @tx_return: Give back an unmodified buffer.
	 */
	void (*tx_return)(struct mcps802154_access *access, int frame_idx,
			  struct sk_buff *skb,
			  enum mcps802154_access_tx_return_reason reason);
};

/**
 * struct mcps802154_access_vendor_ops - Callbacks to implement a vendor
 * specific access.
 *
 * Each callback can return 0 to continue the access, 1 to stop it or an error.
 *
 * If access is stopped, the &mcps802154_access.timestamp_dtu and
 * &mcps802154_access.duration_dtu are used to compute the next access, unless
 * duration is 0, in this case the current date is requested from the driver.
 *
 * In case of error, the devices transition to the broken state.
 *
 * If the callback is missing this is treated like an error, except for
 * &mcps802154_access_vendor_ops.handle and
 * &mcps802154_access_vendor_ops.schedule_change which are ignored.
 */
struct mcps802154_access_vendor_ops {
	/*
	 * Anonymous structure which must be declared at the beginning of all
	 * access ops.
	 */
	struct {
		/**
		 * @access_done: Called when the access is done, successfully or
		 * not.
		 */
		void (*access_done)(struct mcps802154_access *access);
	};
	/**
	 * @handle: Called once to start the access, ignored if NULL.
	 */
	int (*handle)(struct mcps802154_access *access);
	/**
	 * @rx_frame: Called when a frame reception is signaled, error if NULL.
	 */
	int (*rx_frame)(struct mcps802154_access *access);
	/**
	 * @rx_timeout: Called when a reception timeout is signaled, error if NULL.
	 */
	int (*rx_timeout)(struct mcps802154_access *access);
	/**
	 * @rx_error: Called when a reception error is signaled, error if NULL.
	 */
	int (*rx_error)(struct mcps802154_access *access,
			enum mcps802154_rx_error_type error);
	/**
	 * @tx_done: Called when end of transmission is signaled.
	 */
	int (*tx_done)(struct mcps802154_access *access);
	/**
	 * @broken: Called when a unrecoverable error is signaled, error if NULL.
	 */
	int (*broken)(struct mcps802154_access *access);
	/**
	 * @schedule_change: Called to handle schedule change, ignored if NULL.
	 */
	int (*schedule_change)(struct mcps802154_access *access);
};

/**
 * struct mcps802154_region - An open region instance. Region handlers can have
 * private data appended after this structure.
 */
struct mcps802154_region {
	/**
	 * @ops: Callbacks for the region.
	 */
	const struct mcps802154_region_ops *ops;
};

/**
 * struct mcps802154_region_ops - Region callbacks, handle access for a specific
 * region in schedule.
 */
struct mcps802154_region_ops {
	/**
	 * @owner: Module owning this region, should be THIS_MODULE in most
	 * cases.
	 */
	struct module *owner;
	/**
	 * @name: Region name.
	 */
	const char *name;
	/**
	 * @registered_entry: Entry in list of registered regions.
	 */
	struct list_head registered_entry;
	/**
	 * @open: Open an instance of this region, return a new region instance,
	 * or NULL in case of error.
	 */
	struct mcps802154_region *(*open)(struct mcps802154_llhw *llhw);
	/**
	 * @close: Close a region instance.
	 */
	void (*close)(struct mcps802154_region *region);
	/**
	 * @set_parameters: Set region parameters, may be NULL.
	 */
	int (*set_parameters)(struct mcps802154_region *region,
			      const struct nlattr *attrs,
			      struct netlink_ext_ack *extack);
	/**
	 * @call: Call region procedure, may be NULL.
	 */
	int (*call)(struct mcps802154_region *region, u32 call_id,
		    const struct nlattr *attrs, const struct genl_info *info);
	/**
	 * @get_access: Get access for a given region at the given timestamp.
	 * Access is valid until &mcps802154_access_ops.access_done() callback
	 * is called. Return NULL if access is not possible.
	 */
	struct mcps802154_access *(*get_access)(
		struct mcps802154_region *region, u32 next_timestamp_dtu,
		int next_in_region_dtu, int region_duration_dtu);
};

/**
 * struct mcps802154_schedule_update - Context valid during a schedule
 * update.
 */
struct mcps802154_schedule_update {
	/**
	 * @expected_start_timestamp_dtu: Expected start timestamp, based on the
	 * current access date and having the new schedule put right after the
	 * old one.
	 */
	u32 expected_start_timestamp_dtu;
	/**
	 * @start_timestamp_dtu: Date of the schedule start, might be too far in
	 * the past for endless schedule.
	 */
	u32 start_timestamp_dtu;
	/**
	 * @duration_dtu: Schedule duration or 0 for endless schedule. This is
	 * also 0 when the schedule is empty.
	 */
	int duration_dtu;
	/**
	 * @n_regions: Number of regions in the schedule.
	 */
	size_t n_regions;
};

/**
 * struct mcps802154_scheduler - An open scheduler instance. Schedulers can have
 * private data appended after this structure.
 */
struct mcps802154_scheduler {
	/**
	 * @ops: Callbacks for the scheduler.
	 */
	const struct mcps802154_scheduler_ops *ops;
};

/**
 * struct mcps802154_scheduler_ops - Callbacks for schedulers. A scheduler
 * provides a schedule to MCPS and updates it when specific frames are
 * received or schedule is no longer valid.
 */
struct mcps802154_scheduler_ops {
	/**
	 * @owner: Module owning this scheduler, should be THIS_MODULE in most
	 * cases.
	 */
	struct module *owner;
	/**
	 * @name: Scheduler name.
	 */
	const char *name;
	/**
	 * @registered_entry: Entry in list of registered schedulers.
	 */
	struct list_head registered_entry;
	/**
	 * @open: Attach a scheduler to a device.
	 */
	struct mcps802154_scheduler *(*open)(struct mcps802154_llhw *llhw);
	/**
	 * @close: Detach and close a scheduler.
	 */
	void (*close)(struct mcps802154_scheduler *scheduler);
	/**
	 * @set_parameters: Configure the scheduler.
	 */
	int (*set_parameters)(struct mcps802154_scheduler *scheduler,
			      const struct nlattr *attrs,
			      struct netlink_ext_ack *extack);
	/**
	 * @set_region_parameters: Configure the region inside the scheduler.
	 */
	int (*set_region_parameters)(struct mcps802154_scheduler *scheduler,
				     u32 region_id, const char *region_name,
				     const struct nlattr *attrs,
				     struct netlink_ext_ack *extack);
	/**
	 * @call: Call scheduler specific procedure.
	 */
	int (*call)(struct mcps802154_scheduler *scheduler, u32 call_id,
		    const struct nlattr *attrs, const struct genl_info *info);
	/**
	 * @call_region: Call region specific procedure.
	 */
	int (*call_region)(struct mcps802154_scheduler *scheduler,
			   u32 region_id, const char *region_name, u32 call_id,
			   const struct nlattr *attrs,
			   const struct genl_info *info);
	/**
	 * @update_schedule: Called to initialize and update the schedule.
	 */
	int (*update_schedule)(
		struct mcps802154_scheduler *scheduler,
		const struct mcps802154_schedule_update *schedule_update,
		u32 next_timestamp_dtu);
	/**
	 * @ranging_setup: Called to configure ranging. This is a temporary
	 * interface.
	 */
	int (*ranging_setup)(
		struct mcps802154_scheduler *scheduler,
		const struct mcps802154_nl_ranging_request *requests,
		unsigned int n_requests);
};

/**
 * mcps802154_region_register() - Register a region, to be called when your
 * module is loaded.
 * @region_ops: Region to register.
 *
 * Return: 0 or error.
 */
int mcps802154_region_register(struct mcps802154_region_ops *region_ops);

/**
 * mcps802154_region_unregister() - Unregister a region, to be called at module
 * unloading.
 * @region_ops: Region to unregister.
 */
void mcps802154_region_unregister(struct mcps802154_region_ops *region_ops);

/**
 * mcps802154_region_open() - Open a region, and set parameters.
 * @llhw: Low-level device pointer.
 * @name: Name of region to open.
 * @params_attr: Nested attribute containing region parameters, may be NULL.
 * @extack: Extended ACK report structure.
 *
 * Return: The open region or NULL on error.
 */
struct mcps802154_region *
mcps802154_region_open(struct mcps802154_llhw *llhw, const char *name,
		       const struct nlattr *params_attr,
		       struct netlink_ext_ack *extack);

/**
 * mcps802154_region_close() - Close a region.
 * @llhw: Low-level device pointer.
 * @region: Pointer to the open region.
 */
void mcps802154_region_close(struct mcps802154_llhw *llhw,
			     struct mcps802154_region *region);

/**
 * mcps802154_region_set_parameters() - Set parameters of an open region.
 * @llhw: Low-level device pointer.
 * @region: Pointer to the open region.
 * @params_attr: Nested attribute containing region parameters, may be NULL.
 * @extack: Extended ACK report structure.
 *
 * Return: 0 or error.
 */
int mcps802154_region_set_parameters(struct mcps802154_llhw *llhw,
				     struct mcps802154_region *region,
				     const struct nlattr *params_attr,
				     struct netlink_ext_ack *extack);

/**
 * mcps802154_region_call() - Call specific procedure in this region.
 * @llhw: Low-level device pointer.
 * @region: Pointer to the open region.
 * @call_id: Identifier of the procedure, region specific.
 * @params_attr: Nested attribute containing region parameters, may be NULL.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
int mcps802154_region_call(struct mcps802154_llhw *llhw,
			   struct mcps802154_region *region, u32 call_id,
			   const struct nlattr *params_attr,
			   const struct genl_info *info);

/**
 * mcps802154_region_event_alloc_skb() - Allocate buffer to send a notification
 * for a region.
 * @llhw: Low-level device pointer.
 * @region: Pointer to the open region.
 * @call_id: Identifier of the procedure, region specific.
 * @portid: Port identifier of the receiver.
 * @approx_len: Upper bound of the data to be put into the buffer.
 * @gfp: Allocation flags.
 *
 * Return: An allocated and pre-filled buffer, or NULL on error.
 */
struct sk_buff *
mcps802154_region_event_alloc_skb(struct mcps802154_llhw *llhw,
				  struct mcps802154_region *region, u32 call_id,
				  u32 portid, int approx_len, gfp_t gfp);

/**
 * mcps802154_region_event() - Send a previously allocated and filled
 * buffer.
 * @llhw: Low-level device pointer.
 * @skb: Buffer to send.
 *
 * Return: 0 or error.
 */
int mcps802154_region_event(struct mcps802154_llhw *llhw, struct sk_buff *skb);

/**
 * mcps802154_scheduler_register() - Register a scheduler, to be called when
 * your module is loaded.
 * @scheduler_ops: Scheduler to register.
 *
 * Return: 0 or error.
 */
int mcps802154_scheduler_register(
	struct mcps802154_scheduler_ops *scheduler_ops);

/**
 * mcps802154_scheduler_unregister() - Unregister a scheduler, to be called at
 * module unloading.
 * @scheduler_ops: Scheduler to unregister.
 */
void mcps802154_scheduler_unregister(
	struct mcps802154_scheduler_ops *scheduler_ops);

/**
 * mcps802154_schedule_set_start() - Change the currently updated schedule start
 * timestamp.
 * @schedule_update: Schedule update context.
 * @start_timestamp_dtu: New start timestamp.
 *
 * Return: 0 or -EINVAL if arguments are garbage.
 */
int mcps802154_schedule_set_start(
	const struct mcps802154_schedule_update *schedule_update,
	u32 start_timestamp_dtu);

/**
 * mcps802154_schedule_recycle() - Purge or recycle the current schedule.
 * @schedule_update: Schedule update context.
 * @n_keeps: Number of regions to keep from the previous schedule.
 * @last_region_duration_dtu:
 *	Duration of the last region, or MCPS802154_DURATION_NO_CHANGE to keep it
 *	unchanged.
 *
 * Return: 0 or -EINVAL if arguments are garbage.
 */
int mcps802154_schedule_recycle(
	const struct mcps802154_schedule_update *schedule_update,
	size_t n_keeps, int last_region_duration_dtu);

/**
 * mcps802154_schedule_add_region() - Add a new region to the currently updated
 * schedule.
 * @schedule_update: Schedule update context.
 * @region: Region to add.
 * @start_dtu: Region start from the start of the schedule.
 * @duration_dtu: Region duration, or 0 for endless region.
 *
 * Return: 0 or error.
 */
int mcps802154_schedule_add_region(
	const struct mcps802154_schedule_update *schedule_update,
	struct mcps802154_region *region, int start_dtu, int duration_dtu);

/**
 * mcps802154_reschedule() - Request to change access as possible.
 * @llhw: Low-level device pointer.
 *
 * Use this to reevaluate the current access as new data is available. For
 * example, the device may be sleeping, or waiting to receive a frame, and you
 * have a fresh frame to send.
 *
 * Request may be ignored if the device is busy, in which case the current
 * access will be done before the new access is examined.
 */
void mcps802154_reschedule(struct mcps802154_llhw *llhw);

/**
 * mcps802154_schedule_invalidate() - Request to invalidate the schedule.
 * @llhw: Low-level device pointer.
 *
 * FSM mutex should be locked.
 *
 * Invalidate the current schedule, which will result on a schedule change.
 * This API should be called from external modules to force schedule change,
 * when for example some parameters changed.
 */
void mcps802154_schedule_invalidate(struct mcps802154_llhw *llhw);

#endif /* NET_MCPS802154_SCHEDULE_H */
