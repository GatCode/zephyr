#ifndef IO_CODER
#define IO_CODER

#include <zephyr.h>
#include <drivers/gpio.h>

/**
 * If enabled, the logic operates in active low fashion
 */
#define IO_CODER_ACTIVE_LEVEL_LOW

struct io_coder {
	/**
	 * @brief device* containing port0.
	 */
	const struct device *port0;
	/**
	 * @brief device* containing port1.
	 */
	const struct device *port1;
};

/**
 * @brief Pin Setup
 *
 * This function performs the setup of a given pin
 * to a given port.
 *
 * @param port   	  Port of the MCU.
 * @param pin    	  Pin number of the MCU.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_pin(const struct device *port, uint8_t pin);

/**
 * @brief Setup 8 consecutive pins
 *
 * This function performs the setup of 8 consecutive pins
 * to a given port (e.g. P0.01-P0.08).
 *
 * @param port   	  Port of the MCU.
 * @param first_pin   Pin number first pin.
 * @param last_pin    Pin number first pin.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_8_consecutive_pins(const struct device *port, uint8_t first_pin, uint8_t last_pin);

/**
 * @brief Pin Setup of pins 1-8 for a given port
 *
 * This function performs the setup of pins 1-8
 * for a given port.
 *
 * @param port   	  Port of the MCU.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_P0_01_to_P0_08_from_port(const struct device *port);

/**
 * @brief Pin Setup of pins 1-8 for the port 0
 *
 * This function performs the setup of pins 1-8
 * for port 0 of the MCU.
 *
 * @param coder   	  io_coder object.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_P0_01_to_P0_08(struct io_coder *coder);

/**
 * @brief Pin Setup of pins 1-8 for the port 1
 *
 * This function performs the setup of pins 1-8
 * for port 1 of the MCU.
 *
 * @param coder   	  io_coder object.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int setup_P1_01_to_P1_08(struct io_coder *coder);

/**
 * @brief Read 8-bit value of the pins 1-8 from a given port
 *
 * This function performs a read operation of
 * pins 1-8 of the given port. The result contains the 8-bit
 * representation of these pins.
 *
 * @param port   	  Port of the MCU.
 * @param result      Pointer in which the read value will be stored.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int read_P01_to_P08_from_port(const struct device *port, uint8_t *result);

/**
 * @brief Read 8-bit value of P0.01 - P0.08
 *
 * This function performs a read operation of
 * pins 1-8 of port 0. The result contains the 8-bit
 * representation of these pins.
 *
 * @param coder   	  io_coder object.
 * @param result      Pointer in which the read value will be stored.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int read_P0_01_to_P0_08(struct io_coder *coder, uint8_t *result);

/**
 * @brief Read 8-bit value of P1.01 - P1.08
 *
 * This function performs a read operation of
 * pins 1-8 of port 1. The result contains the 8-bit
 * representation of these pins.
 *
 * @param coder   	  io_coder object.
 * @param result      Pointer in which the read value will be stored.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int read_P1_01_to_P1_08(struct io_coder *coder, uint8_t *result);

#endif