#include <zephyr.h>
#include <sys/printk.h>

#include <hw_info.h>

void main(void)
{
	int err;
	uint64_t id;

	err = get_device_id(&id);
	if(err) {
		printk("Error getting id (err %d)\n", err);
	}

	printk("ID: %llX\n", id);
}