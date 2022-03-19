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
};

/**
 * @brief Set specific GPIO port
 *
 * This function sets the io_coder port to the given port.
 *
 * @param coder   	  io_coder object.
 * @param port    	  Pointer to a GPIO port.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int set_port(struct io_coder *coder, const struct device *port);

/**
 * @brief Pin Setup
 *
 * This function performs the setup of a given pin.
 *
 * @param coder   	  io_coder object.
 * @param pin    	  Pin number of the MCU.
 * @param read        Toggle for the read mode configuration.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_pin(struct io_coder *coder, uint8_t pin, bool read);

/**
 * @brief Pin setup of 8 given pins for port 1
 * 
 * This function performs the setup of 8 consecutive pins
 * of port 1 in read or write mode.
 *
 * @param coder   	  io_coder object.
 * @param first_pin   Pin number first pin.
 * @param last_pin    Pin number first pin.
 * @param read   	  Read mode configuration flag.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_8_bit_io(struct io_coder *coder, uint8_t first_pin, uint8_t last_pin, bool read);

/**
 * @brief Read 8-bit value of the given pin range
 * 
 * This function performs a read operation of the given pin range
 * on the port specified in the io_coder object. The result contains
 * the 8-bit representation of of the pin states.
 *
 * @param coder   	  io_coder object.
 * @param first_pin   Pin number first pin.
 * @param last_pin    Pin number first pin.
 * @param result      Pointer in which the read value will be stored.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int read_8_bit(struct io_coder *coder, uint8_t first_pin, uint8_t last_pin, uint8_t *result);
















// /**
//  * @brief Read 8-bit value of the pins 1-8 from a given port
//  *
//  * This function performs a read operation of
//  * pins 1-8 of the given port. The result contains the 8-bit
//  * representation of these pins.
//  *
//  * @param port   	  Port of the MCU.
//  * @param result      Pointer in which the read value will be stored.
//  *
//  * @return Zero on success or (negative) error code otherwise.
//  */
// int read_P01_to_P08_from_port(const struct device *port, uint8_t *result);

// /**
//  * @brief Read 8-bit value of P0.01 - P0.08
//  *
//  * This function performs a read operation of
//  * pins 1-8 of port 0. The result contains the 8-bit
//  * representation of these pins.
//  *
//  * @param coder   	  io_coder object.
//  * @param result      Pointer in which the read value will be stored.
//  *
//  * @return Zero on success or (negative) error code otherwise.
//  */
// int read_P0_01_to_P0_08(struct io_coder *coder, uint8_t *result);

// /**
//  * @brief Read 8-bit value of P1.01 - P1.08
//  *
//  * This function performs a read operation of
//  * pins 1-8 of port 1. The result contains the 8-bit
//  * representation of these pins.
//  *
//  * @param coder   	  io_coder object.
//  * @param result      Pointer in which the read value will be stored.
//  *
//  * @return Zero on success or (negative) error code otherwise.
//  */
// int read_P1_01_to_P1_08(struct io_coder *coder, uint8_t *result);

#endif