#ifndef IO_CODER
#define IO_CODER

#include <zephyr.h>
#include <drivers/gpio.h>

/**
 * If enabled, the logic operates in active low fashion
 */
// #define IO_CODER_ACTIVE_LEVEL_LOW

struct io_coder {
	/**
	 * @brief device* containing the port.
	 */
	const struct device *port;
	/**
	 * @brief gpio_port_pins_t containing all 8 pin definitions.
	 */
	gpio_port_pins_t pins;
};

/**
 * @brief Pin setup of 8 specified pins for port 0
 * 
 * This function is simmilar to setup_8_bit_io with first_pin and last_pin
 * but allows for specific pin declaration.
 *
 * @param coder   	  io_coder object.
 * @param pins	      gpio_port_pins_t containing 8 pin definitions
 * @param port1   	  Port 1 configuration flag.
 * @param read   	  Read mode configuration flag.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_8_bit_io(struct io_coder *coder, uint8_t pins[8], bool port1, bool read);

/**
 * @brief Pin setup of 8 given pins for port 0
 * 
 * This function performs the setup of 8 consecutive pins
 * of port 0 in read or write mode. If the port1 flag is
 * set, the setup will cover the specified pins on port 1.
 *
 * @param coder   	  io_coder object.
 * @param first_pin   Pin number first pin.
 * @param last_pin    Pin number first pin.
 * @param port1   	  Port 1 configuration flag.
 * @param read   	  Read mode configuration flag.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_8_bit_io_consecutive(struct io_coder *coder, uint8_t first_pin, uint8_t last_pin, bool port1, bool read);

/**
 * @brief Read 8-bit value
 * 
 * This function performs a read operation of the set pin range
 * and port specified in the io_coder object. The result contains
 * the 8-bit representation of the pin states.
 *
 * @param coder   	  io_coder object.
 * @param result      Pointer in which the read value will be stored.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int read_8_bit(struct io_coder *coder, uint8_t *result);

/**
 * @brief Write 8-bit value
 * 
 * This function performs a write operation to the set pin range
 * and port specified in the io_coder object.
 *
 * @param coder   	  io_coder object.
 * @param value       uint8_t which gets written to the set pin range
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int write_8_bit(struct io_coder *coder, uint8_t value);

#endif