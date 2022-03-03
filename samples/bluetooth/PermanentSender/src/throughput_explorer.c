#include <throughput_explorer.h>
#include <drivers/timer/nrf_rtc_timer.h>

int u64_to_u8_arr(uint64_t input, uint8_t *output)
{
    for (int i = 0; i < 8; i++) {
        output[i] = input >> (8 * i);
    }

    return 0;
}

int set_id(uint64_t id, struct explorer_payload *payload)
{
    u64_to_u8_arr(id, payload->id);
    
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

int reset_config(struct explorer_config *cfg)
{
    if(cfg == NULL) {
        return -EINVAL;
    }
    
    cfg->packet_id = 0;
    cfg->rtc_offset = z_nrf_rtc_timer_read();

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