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

struct explorer_statistic {
	/**
	 * @brief Total number of received packets.
	 */
	uint64_t recv;
	/**
	 * @brief Total number of lost packets.
	 */
	uint64_t lost;
	/**
	 * @brief Number of lost packets before the current one.
	 */
	uint64_t currently_lost;
	/**
	 * @brief ID of the received packet.
	 */
	uint64_t id;
	/**
	 * @brief Timestamp of the received packet.
	 */
	uint64_t timestamp;
	/**
	 * @brief ID of the last received packet.
	 */
	int64_t last_packet_id;
	/**
	 * @brief Latency of the current packet.
	 */
	uint64_t latency;
};

/** Convert u64 to u8[8] for Bluetooth transmission. */
int u64_to_u8_arr(uint64_t input, uint8_t *output);

/** Convert u8[8] to u64 after Bluetooth transmission. */
int u8_arr_to_u64(uint8_t *input, uint64_t *output);

/** Throughput Explorer Payload Setter - ID. */
int set_id(uint64_t id, struct explorer_payload *payload);

/**
 * @brief Throughput Explorer Payload Getter - ID.
 *
 * This function extracts the id from the payload, converts
 * it to uint64_t and stores this value in the explorer_statistic
 *
 * @param statistic   Throughput Explorer Statistic.
 * @param payload     Throughput Explorer Payload.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int get_id(struct explorer_statistic *statistic, struct explorer_payload *payload);

/** Throughput Explorer Payload Setter - Timestamp. */
int set_timestamp(uint64_t current_ts, struct explorer_config *cfg, struct explorer_payload *payload);

/**
 * @brief Throughput Explorer Payload Getter - Timestamp.
 *
 * This function extracts the timestamp from the payload, converts
 * it to uint64_t and stores this value in the explorer_statistic
 *
 * @param statistic   Throughput Explorer Statistic.
 * @param payload     Throughput Explorer Payload.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int get_timestamp(struct explorer_statistic *statistic, struct explorer_payload *payload);

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

/** Reset Throughput Explorer Statistic. */
int reset_statistic(struct explorer_statistic *statistic);

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

/**
 * @brief Update Throughput Explorer Statistic.
 *
 * This function updates the statistic with the payload contents,
 * calculates the latency and determines the link quality
 *
 * @param statistic   Throughput Explorer Statistic.
 * @param cfg         Throughput Explorer Config.
 * @param payload     Throughput Explorer Payload.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int update_statistic(struct explorer_statistic *statistic, struct explorer_config *cfg, struct explorer_payload *payload);

/**
 * @brief Prints the content of the explorer_statistic.
 *
 * This function visualizes the explorer_statistic.
 *
 * @param statistic   Throughput Explorer Statistic.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int print_statistic(struct explorer_statistic *statistic, uint8_t skip_packets);

#endif