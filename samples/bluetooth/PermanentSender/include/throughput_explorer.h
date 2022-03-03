#ifndef THROUGHPUTEXPLORER_H
#define THROUGHPUTEXPLORER_H

#include <zephyr.h>

struct explorer_config {
	/**
	 * @brief ID of the current packet.
	 *
	 * The packet id is needed to be able to detect the absence
	 * of packets on the receiver side.
	 */
	uint64_t packet_id;
	/**
	 * @brief Time offset between boot and reset_config call.
	 *
	 * Time offset between the MCU boot and the previous call to
	 * reset_config. This timestamp is needed for sender/receiver
	 * synchronization. Allows the RTC to act as a counter starting
	 * at an artificial zero point (rtc_offset).
	 */
	uint64_t rtc_offset;
};

struct explorer_payload {
	/**
	 * @brief u8[8] containing the packet ID.
	 */
	uint8_t id[8];
	/**
	 * @brief u8[8] containing the timestamp.
	 */
	uint8_t timestamp[8];
};

/** Convert u64 to u8[8] for Bluetooth transmission. */
int u64_to_u8_arr(uint64_t input, uint8_t *output);

/** Throughput Explorer Payload Setter - ID. */
int set_id(uint64_t id, struct explorer_payload *payload);

/** Throughput Explorer Payload Setter - Timestamp. */
int set_timestamp(uint64_t current_ts, struct explorer_config *cfg, struct explorer_payload *payload);

/**
 * @brief Reset Throughput Explorer Config.
 *
 * This function sets all member variables of the throughput explorer
 * to zero and sets the rtc_offset to the current RTC timestamp.
 *
 * @param cfg   Throughput Explorer Config.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int reset_config(struct explorer_config *cfg);

/**
 * @brief Update Throughput Explorer Payload.
 *
 * This function updates the payload by increasing the packet_id
 * and setting the current RTC timestamp.
 *
 * @param cfg       Throughput Explorer Config.
 * @param payload   Throughput Explorer Payload.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int update_payload(struct explorer_config *cfg, struct explorer_payload *payload);

#endif