#include <zephyr.h>
#include <drivers/gpio.h>

#include <io_coder.h>

static struct io_coder io_encoder = {0};
static struct io_coder io_decoder = {0};

void main(void)
{
	int err;
	uint8_t reading;

	/* Setup Encoder Pins */
	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	/* Setup Decoder Pins */
	// uint8_t pins[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	// err = setup_8_bit_io(&io_decoder, (uint8_t*)&pins, true, true);
	// if(err) {
	// 	printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	// }

	while(1) {
		for (uint8_t i = 0; i < 256; i++)
		{
			err = write_8_bit(&io_encoder, i);
			if(err) {
				printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
			}

			// err = read_8_bit(&io_decoder, &reading);
			// if(err) {
			// 	printk("Error reading 8bit value from P1.01 - P1.08 (err %d)\n", err);
			// }
			// printk("READ: %u\n", reading);

			k_sleep(K_MSEC(1000));
		}
	}
}