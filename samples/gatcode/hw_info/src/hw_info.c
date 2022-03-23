#include <zephyr.h>
#include <hw_info.h>
#include <drivers/hwinfo.h>

int get_device_id(uint64_t *id)
{
    if(id == NULL) {
        return -EINVAL;
    }

    static uint8_t hw_id[8];
    int err = hwinfo_get_device_id(hw_id, sizeof(hw_id));
    if(err < 0) {
		return err;
	}

    *id =
    (uint64_t)hw_id[0] << 0 * 8 |
    (uint64_t)hw_id[1] << 1 * 8 |
    (uint64_t)hw_id[2] << 2 * 8 |
    (uint64_t)hw_id[3] << 3 * 8 |
    (uint64_t)hw_id[4] << 4 * 8 |
    (uint64_t)hw_id[5] << 5 * 8 |
    (uint64_t)hw_id[6] << 6 * 8 |
    (uint64_t)hw_id[7] << 7 * 8;

    return 0;
}