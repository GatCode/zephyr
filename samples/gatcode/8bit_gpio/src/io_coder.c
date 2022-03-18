#include <io_coder.h>

int setup_pin(const struct device *port, uint8_t pin)
{
    if(port == NULL) {
        return -EINVAL;
    }

    int err = gpio_pin_configure(port, pin, 1);
    if(err) {
        return err;
    }

    return 0;
}

int setup_8_consecutive_pins(const struct device *port, uint8_t first_pin, uint8_t last_pin)
{
    if(first_pin > last_pin || last_pin - first_pin != 7 || first_pin < 0 || last_pin < 0) {
        return -EINVAL;
    }

    int err;
    for(uint8_t i = first_pin; i <= last_pin; i++) {
		err = setup_pin(port, i);
        if(err) {
            return err;
        }
	}

    return 0;
}

int setup_P0_01_to_P0_08(struct io_coder *coder)
{
    if(coder == NULL) {
        return -EINVAL;
    }

    coder->port0 = device_get_binding("GPIO_0");
    if(coder->port0 == NULL) {
        return -EINVAL;
    }

    return setup_8_consecutive_pins(coder->port0, 1, 8);
}

int setup_P1_01_to_P1_08(struct io_coder *coder)
{
    if(coder == NULL) {
        return -EINVAL;
    }

    coder->port1 = device_get_binding("GPIO_1");
    if(coder->port1 == NULL) {
        return -EINVAL;
    }

    return setup_8_consecutive_pins(coder->port1, 33, 40);
}

int read_P01_to_P08_from_port(const struct device *port, uint8_t *result)
{
    gpio_port_value_t v;
    
    int err = gpio_port_get(port, &v);
    if(err) {
        return err;
    }

    #ifdef IO_CODER_ACTIVE_LEVEL_LOW
        *result = ~v >> 1;
    #else
        *result = (v & (~BIT(1) | ~BIT(2) | ~BIT(3) | ~BIT(4) | ~BIT(5) | ~BIT(6) | ~BIT(7) | ~BIT(8))) >> 1;
    #endif

    return 0;
}

int read_P0_01_to_P0_08(struct io_coder *coder, uint8_t *result)
{
    return read_P01_to_P08_from_port(coder->port0, result);
}

int read_P1_01_to_P1_08(struct io_coder *coder, uint8_t *result)
{
    return read_P01_to_P08_from_port(coder->port1, result);
}