#include <io_coder.h>

int set_port(struct io_coder *coder, const struct device *port)
{
    if(coder == NULL || port == NULL) {
        return -EINVAL;
    }

    coder->port = port;
    return 0;
}

int setup_pin(struct io_coder *coder, uint8_t pin, bool read)
{
    if(coder->port == NULL) {
        return -EINVAL;
    }

    int err = gpio_pin_configure(coder->port, pin, read ? GPIO_INPUT : GPIO_OUTPUT);
    if(err) {
        return err;
    }

    return 0;
}

int setup_8_bit_io(struct io_coder *coder, uint8_t first_pin, uint8_t last_pin, bool read)
{
    if(coder == NULL || first_pin > last_pin || last_pin - first_pin != 7 || first_pin < 0 || last_pin < 0) {
        return -EINVAL;
    }

    coder->port = device_get_binding("GPIO_1");
    if(coder->port == NULL) {
        return -EINVAL;
    }

    int err;
    for(uint8_t i = first_pin; i <= last_pin; i++) {
		err = setup_pin(coder, i, read);
        if(err) {
            return err;
        }
	}

    return 0;
}

int read_8_bit(struct io_coder *coder, uint8_t first_pin, uint8_t last_pin, uint8_t *result)
{
    gpio_port_value_t value;
    
    int err = gpio_port_get(coder->port, &value);
    if(err) {
        return err;
    }

    #ifdef IO_CODER_ACTIVE_LEVEL_LOW
        *result = ~value >> 1;
    #else
        uint32_t mask = 0;
        for (uint8_t i = first_pin; i <= last_pin; i++)
        {
            mask |= ~BIT(i);
        }
        *result = (value & mask) >> 1;
    #endif

    return 0;
}