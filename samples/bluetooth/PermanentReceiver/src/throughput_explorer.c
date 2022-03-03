#include <throughput_explorer.h>
#include <drivers/timer/nrf_rtc_timer.h>

int u64_to_u8_arr(uint64_t input, uint8_t *output)
{
    if(output == NULL) {
        return -EINVAL;
    }

    for (int i = 0; i < 8; i++) {
        output[i] = input >> (8 * i);
    }

    return 0;
}

int u8_arr_to_u64(uint8_t *input, uint64_t *output)
{
    if(input == NULL || output == NULL) {
        return -EINVAL;
    }

    *output =
    (uint64_t)input[0] << 0 * 8 |
    (uint64_t)input[1] << 1 * 8 |
    (uint64_t)input[2] << 2 * 8 |
    (uint64_t)input[3] << 3 * 8 |
    (uint64_t)input[4] << 4 * 8 |
    (uint64_t)input[5] << 5 * 8 |
    (uint64_t)input[6] << 6 * 8 |
    (uint64_t)input[7] << 7 * 8;

    return 0;
}

int set_id(uint64_t id, struct explorer_payload *payload)
{
    if(payload == NULL) {
        return -EINVAL;
    }

    u64_to_u8_arr(id, payload->id);
    
    return 0;
}

int get_id(struct explorer_statistic *statistic, struct explorer_payload *payload)
{
    if(statistic == NULL || payload == NULL) {
        return -EINVAL;
    }

    u8_arr_to_u64(payload->id, &statistic->id);

    return 0;
}


int set_timestamp(uint64_t current_ts, struct explorer_config *cfg, struct explorer_payload *payload)
{
    if(cfg == NULL || payload == NULL) {
        return -EINVAL;
    }
        
    u64_to_u8_arr(current_ts - cfg->rtc_offset, payload->timestamp);

    return 0;
}

int get_timestamp(struct explorer_statistic *statistic, struct explorer_payload *payload)
{
    if(statistic == NULL || payload == NULL) {
        return -EINVAL;
    }

    u8_arr_to_u64(payload->timestamp, &statistic->timestamp);

    return 0;
}

int reset_config(struct explorer_config *cfg)
{
    if(cfg == NULL) {
        return -EINVAL;
    }
    
    cfg->packet_id = 0;
    cfg->rtc_offset = z_nrf_rtc_timer_read();

    return 0;
}

int reset_statistic(struct explorer_statistic *statistic)
{
    if(statistic == NULL) {
        return -EINVAL;
    }
    
    statistic->recv = 0;
    statistic->lost = 0;
    statistic->currently_lost = 0;
    statistic->id = 0;
    statistic->timestamp = 0;
    statistic->last_packet_id = 0;
    statistic->latency = 0;

    return 0;
}

int update_payload(struct explorer_config *cfg, struct explorer_payload *payload)
{
    if(cfg == NULL || payload == NULL) {
        return -EINVAL;
    }
    
    set_id(cfg->packet_id++, payload);
	set_timestamp(z_nrf_rtc_timer_read(), cfg, payload);

    return 0;
}

int update_statistic(struct explorer_statistic *statistic, struct explorer_config *cfg, struct explorer_payload *payload)
{
    if(statistic == NULL || cfg == NULL || payload == NULL) {
        return -EINVAL;
    }

    get_id(statistic, payload);
    get_timestamp(statistic, payload);
    statistic->latency = z_nrf_rtc_timer_read() - cfg->rtc_offset - statistic->timestamp;

    if(statistic->id != statistic->last_packet_id + 1) {
        statistic->currently_lost = statistic->id - statistic->last_packet_id;
        statistic->lost += statistic->currently_lost;
    } else {
        statistic->recv++;
    }

    statistic->last_packet_id = statistic->id;

    return 0;
}

int print_statistic(struct explorer_statistic *statistic, uint8_t skip_packets)
{
    if(statistic == NULL) {
        return -EINVAL;
    }

	if((statistic->recv + statistic->lost) % skip_packets == 0) {
		printk("Received: %llu/%llu (%.2f%%) - Latency %.2f ms\n",
			statistic->recv, statistic->recv + statistic->lost,
			(float)statistic->recv * 100 / (statistic->recv + statistic->lost),
            k_ticks_to_us_floor64(statistic->latency) / 1000.0);
	}

    return 0;
}