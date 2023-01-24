#include <io_coder.h>

int setup_8_bit_io(struct io_coder *coder, uint8_t pins[8], bool port1, bool read)
{
    if(coder == NULL) {
        return -EINVAL;
    }

    if(port1) {
        coder->port = device_get_binding("GPIO_1");
    } else {
        coder->port = device_get_binding("GPIO_0");
    }
    if(coder->port == NULL) {
        return -EINVAL;
    }

    int err;
    for(uint8_t i = 0; i < 8; i++) {
        coder->pins |= BIT(pins[i]);
		err = gpio_pin_configure(coder->port, pins[i], read ? GPIO_INPUT : GPIO_OUTPUT);
        if(err) {
            return err;
        }
	}

    return 0;
}

int setup_8_bit_io_consecutive(struct io_coder *coder, uint8_t first_pin, uint8_t last_pin, bool port1, bool read)
{
    if(coder == NULL || first_pin > last_pin || last_pin - first_pin != 7 || first_pin < 0 || last_pin < 0) {
        return -EINVAL;
    }

    uint8_t pins[8];
    uint8_t index = 0;
    for(uint8_t pin = first_pin; pin <= last_pin; pin++) {
		pins[index++] = pin;
	}

    return setup_8_bit_io(coder, pins, port1, read);
}

int read_8_bit(struct io_coder *coder, uint8_t *result)
{
    if(coder == NULL || result == NULL) {
        return -EINVAL;
    }

    gpio_port_value_t value;
    
    int err = gpio_port_get(coder->port, &value);
    if(err) {
        return err;
    }

    #ifdef IO_CODER_ACTIVE_LEVEL_LOW
        *result = ~value >> 1;
    #else
        *result = (value & coder->pins) >> 1;
    #endif

    return 0;
}

int write_8_bit(struct io_coder *coder, uint8_t value)
{
    if(coder == NULL) {
        return -EINVAL;
    }

    gpio_port_set_clr_bits(coder->port, value << 1, ~(value << 1));

    return 0;
}