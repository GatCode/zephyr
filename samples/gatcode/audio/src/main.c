/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/audio/audio.h>
#include <sys/byteorder.h>
#include <io_coder.h>
#include <hw_info.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* D-Cube Defines */
/* ------------------------------------------------------ */
#define REMOTE true
#define SENDER_REMOTE remote_201
#define SENDER_LOCAL local_42
#define SENDER_START_DELAY_MS 25000

/* ------------------------------------------------------ */
/* Sender Specific */
/* ------------------------------------------------------ */

/* When BROADCAST_ENQUEUE_COUNT > 1 we can enqueue enough buffers to ensure that
 * the controller is never idle
 */
#define BROADCAST_ENQUEUE_COUNT 2U
#define TOTAL_BUF_NEEDED (BROADCAST_ENQUEUE_COUNT * CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT)

BUILD_ASSERT(CONFIG_BT_ISO_TX_BUF_COUNT >= TOTAL_BUF_NEEDED,
	     "CONFIG_BT_ISO_TX_BUF_COUNT should be at least "
	     "BROADCAST_ENQUEUE_COUNT * CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT");

#define BT_AUDIO_LC3_BROADCAST_PRESET_CUSTOM \
	BT_AUDIO_LC3_PRESET( \
		BT_CODEC_LC3_CONFIG_8_2, \
		BT_CODEC_LC3_QOS_10_INOUT_UNFRAMED(30u, 2u, 10u, 40000u) \
	)

static struct bt_audio_lc3_preset preset_8_2_1 = BT_AUDIO_LC3_BROADCAST_PRESET_CUSTOM;
static struct bt_audio_stream streams_sender[CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT];
static struct bt_audio_broadcast_source *broadcast_source;

NET_BUF_POOL_FIXED_DEFINE(tx_pool,
			  TOTAL_BUF_NEEDED,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
static uint8_t mock_data[CONFIG_BT_ISO_TX_MTU];
static bool stopping;

static K_SEM_DEFINE(sem_started, 0U, ARRAY_SIZE(streams_sender));
static K_SEM_DEFINE(sem_stopped, 0U, ARRAY_SIZE(streams_sender));

static void stream_started_cb_sender(struct bt_audio_stream *stream)
{
	k_sem_give(&sem_started);
}

static void stream_stopped_cb_sender(struct bt_audio_stream *stream)
{
	k_sem_give(&sem_stopped);
}

static void stream_sent_cb(struct bt_audio_stream *stream)
{
	static uint32_t sent_cnt;
	struct net_buf *buf;
	int ret;

	if (stopping) {
		return;
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	if (buf == NULL) {
		printk("Could not allocate buffer when sending on %p\n",
		       stream);
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, mock_data, preset_8_2_1.qos.sdu);
	ret = bt_audio_stream_send(stream, buf);
	if (ret < 0) {
		/* This will end broadcasting on this stream. */
		printk("Unable to broadcast data on %p: %d\n", stream, ret);
		net_buf_unref(buf);
		return;
	}

	int err = write_8_bit(&io_encoder, sent_cnt % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}

	sent_cnt++;
	if ((sent_cnt % 1000U) == 0U) {
		printk("Sent %u total ISO packets\n", sent_cnt);
	}
}

struct bt_audio_stream_ops stream_ops_sender = {
	.started = stream_started_cb_sender,
	.stopped = stream_stopped_cb_sender,
	.sent = stream_sent_cb
};

/* ------------------------------------------------------ */
/* Receiver Specific */
/* ------------------------------------------------------ */

#define SEM_TIMEOUT K_SECONDS(60)

static K_SEM_DEFINE(sem_broadcaster_found, 0U, 1U);
static K_SEM_DEFINE(sem_pa_synced, 0U, 1U);
static K_SEM_DEFINE(sem_base_received, 0U, 1U);
static K_SEM_DEFINE(sem_syncable, 0U, 1U);
static K_SEM_DEFINE(sem_pa_sync_lost, 0U, 1U);

static struct bt_audio_broadcast_sink *broadcast_sink;
static struct bt_audio_stream streams[CONFIG_BT_AUDIO_BROADCAST_SNK_STREAM_COUNT];

/* Create a mask for the maximum BIS we can sync to using the number of streams
 * we have. We add an additional 1 since the bis indexes start from 1 and not
 * 0.
 */
static const uint32_t bis_index_mask = BIT_MASK(ARRAY_SIZE(streams) + 1U);
static uint32_t bis_index_bitfield;

static void stream_started_cb_recv(struct bt_audio_stream *stream)
{
	printk("Stream %p started\n", stream);
}

static void stream_stopped_cb_recv(struct bt_audio_stream *stream)
{
	printk("Stream %p stopped\n", stream);
}

static void stream_recv_cb(struct bt_audio_stream *stream, struct net_buf *buf)
{
	static uint32_t recv_cnt;

	int err = write_8_bit(&io_encoder, recv_cnt % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}

	recv_cnt++;
	if ((recv_cnt % 1000U) == 0U) {
		printk("Received %u total ISO packets\n", recv_cnt);
	}
}

struct bt_audio_stream_ops stream_ops_recv = {
	.started = stream_started_cb_recv,
	.stopped = stream_stopped_cb_recv,
	.recv = stream_recv_cb
};

static bool scan_recv_cb(const struct bt_le_scan_recv_info *info,
			 uint32_t broadcast_id)
{
	k_sem_give(&sem_broadcaster_found);

	return true;
}

static void scan_term_cb(int err)
{
	if (err != 0) {
		printk("Scan terminated with error: %d\n", err);
	}
}

static void pa_synced_cb(struct bt_audio_broadcast_sink *sink,
			 struct bt_le_per_adv_sync *sync,
			 uint32_t broadcast_id)
{
	if (broadcast_sink != NULL) {
		printk("Unexpected PA sync\n");
		return;
	}

	printk("PA synced for broadcast sink %p with broadcast ID 0x%06X\n",
	       sink, broadcast_id);

	broadcast_sink = sink;

	k_sem_give(&sem_pa_synced);
}

static void base_recv_cb(struct bt_audio_broadcast_sink *sink,
			 const struct bt_audio_base *base)
{
	uint32_t base_bis_index_bitfield = 0U;

	if (k_sem_count_get(&sem_base_received) != 0U) {
		return;
	}

	printk("Received BASE with %u subgroups from broadcast sink %p\n",
	       base->subgroup_count, sink);

	for (size_t i = 0U; i < base->subgroup_count; i++) {
		for (size_t j = 0U; j < base->subgroups[i].bis_count; j++) {
			const uint8_t index = base->subgroups[i].bis_data[j].index;

			base_bis_index_bitfield |= BIT(index);
		}
	}

	bis_index_bitfield = base_bis_index_bitfield & bis_index_mask;

	k_sem_give(&sem_base_received);
}

static void syncable_cb(struct bt_audio_broadcast_sink *sink, bool encrypted)
{
	if (encrypted) {
		printk("Cannot sync to encrypted broadcast source\n");
		return;
	}

	k_sem_give(&sem_syncable);
}

static void pa_sync_lost_cb(struct bt_audio_broadcast_sink *sink)
{
	if (broadcast_sink == NULL) {
		printk("Unexpected PA sync lost\n");
		return;
	}

	printk("Sink %p disconnected\n", sink);

	broadcast_sink = NULL;

	k_sem_give(&sem_pa_sync_lost);
}

static struct bt_audio_broadcast_sink_cb broadcast_sink_cbs = {
	.scan_recv = scan_recv_cb,
	.scan_term = scan_term_cb,
	.base_recv = base_recv_cb,
	.syncable = syncable_cb,
	.pa_synced = pa_synced_cb,
	.pa_sync_lost = pa_sync_lost_cb
};

static int init(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth enable failed (err %d)\n", err);
		return err;
	}

	printk("Bluetooth initialized\n");

	bt_audio_broadcast_sink_register_cb(&broadcast_sink_cbs);

	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
		streams[i].ops = &stream_ops_recv;
	}

	return 0;
}

static void reset(void)
{
	bis_index_bitfield = 0U;

	k_sem_reset(&sem_broadcaster_found);
	k_sem_reset(&sem_pa_synced);
	k_sem_reset(&sem_base_received);
	k_sem_reset(&sem_syncable);
	k_sem_reset(&sem_pa_sync_lost);

	if (broadcast_sink != NULL) {
		bt_audio_broadcast_sink_delete(broadcast_sink);
		broadcast_sink = NULL;
	}
}

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */

void main(void)
{
	int err;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	uint64_t id;
	err = get_device_id(&id);
	if(err) {
		printk("Error getting id (err %d)\n", err);
	}

	if(id == (REMOTE == true ? SENDER_REMOTE : SENDER_LOCAL)) { // sender
		err = bt_enable(NULL);
		if (err) {
			printk("Bluetooth init failed (err %d)\n", err);
			return;
		}
		printk("Bluetooth initialized\n");

		for (size_t i = 0U; i < ARRAY_SIZE(streams_sender); i++) {
			streams_sender[i].ops = &stream_ops_sender;
		}

		for (size_t i = 0U; i < ARRAY_SIZE(mock_data); i++) {
			/* Initialize mock data */
			mock_data[i] = i;
		}

		while (true) {
			printk("Creating broadcast source\n");
			err = bt_audio_broadcast_source_create(streams_sender,
								ARRAY_SIZE(streams_sender),
								&preset_8_2_1.codec,
								&preset_8_2_1.qos,
								&broadcast_source);
			if (err != 0) {
				printk("Unable to create broadcast source: %d\n", err);
				return;
			}

			printk("Starting broadcast source\n");
			stopping = false;
			err = bt_audio_broadcast_source_start(broadcast_source);
			if (err != 0) {
				printk("Unable to start broadcast source: %d\n", err);
				return;
			}

			/* Wait for all to be started */
			for (size_t i = 0U; i < ARRAY_SIZE(streams_sender); i++) {
				k_sem_take(&sem_started, K_FOREVER);
			}
			printk("Broadcast source started\n");

			if(REMOTE) {
				k_sleep(K_MSEC(SENDER_START_DELAY_MS));
			}

			/* Initialize sending */
			for (size_t i = 0U; i < ARRAY_SIZE(streams_sender); i++) {
				for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
					stream_sent_cb(&streams_sender[i]);
				}
			}

			while(true) {}

			// printk("Waiting %u seconds before stopped\n",
			//        BROADCAST_SOURCE_LIFETIME);
			// k_sleep(K_SECONDS(BROADCAST_SOURCE_LIFETIME));

			// printk("Stopping broadcast source\n");
			// stopping = true;
			// err = bt_audio_broadcast_source_stop(broadcast_source);
			// if (err != 0) {
			// 	printk("Unable to stop broadcast source: %d\n", err);
			// 	return;
			// }

			// /* Wait for all to be stopped */
			// for (size_t i = 0U; i < ARRAY_SIZE(streams_sender); i++) {
			// 	k_sem_take(&sem_stopped, K_FOREVER);
			// }
			// printk("Broadcast source stopped\n");

			// printk("Deleting broadcast source\n");
			// err = bt_audio_broadcast_source_delete(broadcast_source);
			// if (err != 0) {
			// 	printk("Unable to delete broadcast source: %d\n", err);
			// 	return;
			// }
			// printk("Broadcast source deleted\n");
			// broadcast_source = NULL;
		}
	} else {
		err = init();
		if (err) {
			printk("Init failed (err %d)\n", err);
			return;
		}

		while (true) {
			reset();

			printk("Scanning for broadcast sources\n");
			err = bt_audio_broadcast_sink_scan_start(BT_LE_SCAN_ACTIVE);
			if (err != 0) {
				printk("Unable to start scan for broadcast sources: %d\n",
					err);
				return;
			}

			/* TODO: Update K_FOREVER with a sane value, and handle error */
			err = k_sem_take(&sem_broadcaster_found, SEM_TIMEOUT);
			if (err != 0) {
				printk("sem_broadcaster_found timed out, resetting\n");
				continue;
			}
			printk("Broadcast source found, waiting for PA sync\n");

			k_sem_take(&sem_pa_synced, SEM_TIMEOUT);
			if (err != 0) {
				printk("sem_pa_synced timed out, resetting\n");
				continue;
			}
			printk("Broadcast source PA synced, waiting for BASE\n");

			k_sem_take(&sem_base_received, SEM_TIMEOUT);
			if (err != 0) {
				printk("sem_base_received timed out, resetting\n");
				continue;
			}
			printk("BASE received, waiting for syncable\n");

			k_sem_take(&sem_syncable, SEM_TIMEOUT);
			if (err != 0) {
				printk("sem_syncable timed out, resetting\n");
				continue;
			}

			printk("Syncing to broadcast\n");
			err = bt_audio_broadcast_sink_sync(broadcast_sink,
							bis_index_bitfield,
							streams,
							&preset_8_2_1.codec, NULL);
			if (err != 0) {
				printk("Unable to sync to broadcast source: %d\n", err);
				return;
			}

			printk("Waiting for PA disconnected\n");
			k_sem_take(&sem_pa_sync_lost, K_FOREVER);
		}
	}	
}
