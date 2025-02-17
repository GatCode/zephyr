/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This is mainly a parse test that verifies that Zephyr header files
 * compile in C++ mode.
 */

#include <string.h>
#include <zephyr/types.h>
#include <stdbool.h>

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/net/buf.h>
/* #include <zephyr/sys/byteorder.h> conflicts with __bswapXX on native_posix */
#include <zephyr/sys/crc.h>
#include <zephyr/sys/crc.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/drivers/cache.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/ec_host_cmd_periph.h>
#include <zephyr/drivers/edac.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/espi_emul.h>
#include <zephyr/drivers/espi.h>
/* drivers/espi_saf.h requires SoC specific header */
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/fpga.h>
#include <zephyr/drivers/gna.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/peci.h>
/* drivers/pinctrl.h requires SoC specific header */
#include <zephyr/drivers/pm_cpu_ops.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
/* drivers/reset.h conflicts with assert() for certain platforms */
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/watchdog.h>

#include <zephyr/ztest.h>

class foo_class {
public:
	foo_class(int foo) : foo(foo) {}
	int get_foo() const { return foo;}
private:
	int foo;
};

struct foo {
	int v1;
};
/* Check that BUILD_ASSERT compiles. */
BUILD_ASSERT(sizeof(foo) == sizeof(int));

static struct foo foos[5];
/* Check that ARRAY_SIZE compiles. */
BUILD_ASSERT(ARRAY_SIZE(foos) == 5, "expected 5 elements");

/* Check that SYS_INIT() compiles. */
static int test_init(const struct device *dev)
{
	return 0;
}

SYS_INIT(test_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* Check that global static object constructors are called. */
foo_class static_foo(12345678);

ZTEST(cxx_tests, test_global_static_ctor)
{
	zassert_equal(static_foo.get_foo(), 12345678);
}

/*
 * Check that dynamic memory allocation (usually, the C library heap) is
 * functional when the global static object constructors are called.
 */
foo_class *static_init_dynamic_foo = new foo_class(87654321);

ZTEST(cxx_tests, test_global_static_ctor_dynmem)
{
	zassert_equal(static_init_dynamic_foo->get_foo(), 87654321);
}

ZTEST(cxx_tests, test_new_delete)
{
	foo_class *test_foo = new foo_class(10);
	zassert_equal(test_foo->get_foo(), 10);
	delete test_foo;
}
ZTEST_SUITE(cxx_tests, NULL, NULL, NULL, NULL, NULL);
