#include <zephyr.h>
#include <drivers/gpio.h>

#include <io_coder.h>

static struct io_coder io_8bit_coder = {0};

const struct device *port1;

void main(void)
{
    port1 = device_get_binding("GPIO_1");

	gpio_pin_configure(port1, 1, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 2, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 3, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 4, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 5, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 6, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 7, GPIO_OUTPUT_LOW);
	gpio_pin_configure(port1, 8, GPIO_OUTPUT_LOW);

    gpio_port_pins_t mask = BIT(1) | BIT(2);
    int err = gpio_port_set_masked(port1, mask, 1);
    if(err) {
        printk("Error setting gpio_port_set_masked (err %d)\n", err);
    }

    while(1) {
		gpio_port_pins_t set = BIT(1) | BIT(3) | BIT(5) | BIT(7);
	  	gpio_port_pins_t clear = BIT(2) | BIT(4) | BIT(6) | BIT(8);

		gpio_port_set_clr_bits(port1, set, clear);
		k_sleep(K_MSEC(1000));

		gpio_port_set_clr_bits(port1, clear, set);
		k_sleep(K_MSEC(1000));
	}
}


// void main(void)
// {
// 	int err;
// 	uint8_t p0_01_to_p0_08_reading, p1_01_to_p1_08_reading;

// 	err = setup_P0_01_to_P0_08(&io_8bit_coder);
// 	if(err) {
// 		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
// 	}

// 	err = setup_P1_01_to_P1_08(&io_8bit_coder);
// 	if(err) {
// 		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
// 	}

// 	while(1) {
// 		err = read_P0_01_to_P0_08(&io_8bit_coder, &p0_01_to_p0_08_reading);
// 		if(err) {
// 			printk("Error reading 8bit value from P1.01 - P1.08 (err %d)\n", err);
// 		}

// 		err = read_P1_01_to_P1_08(&io_8bit_coder, &p1_01_to_p1_08_reading);
// 		if(err) {
// 			printk("Error reading 8bit value from P1.01 - P1.08 (err %d)\n", err);
// 		}

// 		printk("P0 1-8: %u | P1 1-8: %u\n", p0_01_to_p0_08_reading, p1_01_to_p1_08_reading);

// 		k_msleep(1000);
// 	}
// }