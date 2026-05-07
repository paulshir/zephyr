/*
 * Copyright (c) 2022, Yonatan Schachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include <stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include <hardware/pio.h>
#include <hardware/clocks.h>

#define DT_DRV_COMPAT raspberrypi_pico_uart_pio

#define CYCLES_PER_BIT 8
#define SIDESET_BIT_COUNT 2

#define PIO_INTERRUPT_SOURCE_REL(source, sm) ((source & ~0x3) + sm)
#define PIO_SET_INTERRUPT_SOURCE_REL(pio, config, source, sm, enabled) ()

struct pio_uart_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t tx_pin;
	const uint32_t rx_pin;
	uint32_t baudrate;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_config_func_t irq_config_func;
	size_t interrupt_index;
#endif
};

struct pio_uart_data {
	size_t tx_sm;
	size_t rx_sm;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_callback_user_data_t irq_cb;
	void *user_data;
	bool irq_tx_enabled;
	bool irq_rx_enabled;
#endif

};

RPI_PICO_PIO_DEFINE_PROGRAM(uart_tx, 0, 3,
		/* .wrap_target */
	0x9fa0, /*  0: pull   block           side 1 [7]  */
	0xf727, /*  1: set    x, 7            side 0 [7]  */
	0x6001, /*  2: out    pins, 1                     */
	0x0642, /*  3: jmp    x--, 2                 [6]  */
		/* .wrap */
);

RPI_PICO_PIO_DEFINE_PROGRAM(uart_rx, 1, 8,
	0x20a0, /*  0: wait   1 pin, 0                    */
		/*  .wrap_target */
	0x2020, /*  1: wait   0 pin, 0                    */
	0xea27, /*  2: set    x, 7                   [10] */
	0x4001, /*  3: in     pins, 1                     */
	0x0643, /*  4: jmp    x--, 3                 [6]  */
	0x00c8, /*  5: jmp    pin, 8                      */
	0xc014, /*  6: irq    nowait 4 rel                */
	0x0000, /*  7: jmp    0                           */
	0x8020, /*  8: push   block                       */
		/*  .wrap */
);

static int pio_uart_tx_init(PIO pio, uint32_t sm, uint32_t tx_pin, float div)
{
	uint32_t offset;
	pio_sm_config sm_config;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(uart_tx))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(uart_tx));
	sm_config = pio_get_default_sm_config();

	sm_config_set_sideset(&sm_config, SIDESET_BIT_COUNT, true, false);
	sm_config_set_out_shift(&sm_config, true, false, 0);
	sm_config_set_out_pins(&sm_config, tx_pin, 1);
	sm_config_set_sideset_pins(&sm_config, tx_pin);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	sm_config_set_clkdiv(&sm_config, div);
	sm_config_set_wrap(&sm_config,
			   offset + RPI_PICO_PIO_GET_WRAP_TARGET(uart_tx),
			   offset + RPI_PICO_PIO_GET_WRAP(uart_tx));

	pio_sm_set_pins_with_mask(pio, sm, BIT(tx_pin), BIT(tx_pin));
	pio_sm_set_pindirs_with_mask(pio, sm, BIT(tx_pin), BIT(tx_pin));
	pio_sm_init(pio, sm, offset, &sm_config);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static int pio_uart_rx_init(PIO pio, uint32_t sm, uint32_t rx_pin, float div)
{
	pio_sm_config sm_config;
	uint32_t offset;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(uart_rx))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(uart_rx));
	sm_config = pio_get_default_sm_config();

	pio_sm_set_consecutive_pindirs(pio, sm, rx_pin, 1, false);
	sm_config_set_in_pins(&sm_config, rx_pin);
	sm_config_set_jmp_pin(&sm_config, rx_pin);
	sm_config_set_in_shift(&sm_config, true, false, 0);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
	sm_config_set_clkdiv(&sm_config, div);
	sm_config_set_wrap(&sm_config,
			   offset + RPI_PICO_PIO_GET_WRAP_TARGET(uart_rx),
			   offset + RPI_PICO_PIO_GET_WRAP(uart_rx));

	pio_sm_init(pio, sm, offset, &sm_config);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static int pio_uart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;
	io_rw_8 *uart_rx_fifo_msb;

	/*
	 * The rx FIFO is 4 bytes wide, add 3 to get the most significant
	 * byte.
	 */
	uart_rx_fifo_msb = (io_rw_8 *)&pio->rxf[data->rx_sm] + 3;
	if (pio_sm_is_rx_fifo_empty(pio, data->rx_sm)) {
		return -1;
	}

	/* Accessing the FIFO pops the read word from it */
	*c = (char)*uart_rx_fifo_msb;
	return 0;
}

static void pio_uart_poll_out(const struct device *dev, unsigned char c)
{
	const struct pio_uart_config *config = dev->config;
	struct pio_uart_data *data = dev->data;

	pio_sm_put_blocking(pio_rpi_pico_get_pio(config->piodev), data->tx_sm, (uint32_t)c);
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int pio_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	int written = 0;
	while (!pio_sm_is_tx_fifo_full(pio, data->tx_sm) && written < size) {
		pio_sm_put(pio, data->tx_sm, (uint32_t) tx_data[written]);
		written++;
	}

	return written;
}

static int pio_fifo_read(const struct device *dev, uint8_t *rx_data, const int size) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;
	io_rw_8 *uart_rx_fifo_msb;

	int read = 0;
	while (!pio_sm_is_rx_fifo_empty(pio, data->rx_sm) && read < size) {
		uart_rx_fifo_msb = (io_rw_8 *)&pio->rxf[data->rx_sm] + 3;
		rx_data[read] = (char)*uart_rx_fifo_msb;
		read++;
	}

	return read;
}

static void pio_irq_tx_enable(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	data->irq_tx_enabled = true;
	pio_set_irqn_source_enabled(pio,
	                            config->interrupt_index,
	                            PIO_INTERRUPT_SOURCE_REL(pis_sm0_tx_fifo_not_full, data->tx_sm),
	                            true);
}

static void pio_irq_tx_disable(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	data->irq_tx_enabled = false;
	pio_set_irqn_source_enabled(pio,
	                            config->interrupt_index,
	                            PIO_INTERRUPT_SOURCE_REL(pis_sm0_tx_fifo_not_full, data->tx_sm),
	                            false);
}

static int pio_irq_tx_ready(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	return !pio_sm_is_tx_fifo_full(pio, data->tx_sm);
}

static int pio_irq_tx_complete(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	return pio_sm_is_tx_fifo_empty(pio, data->tx_sm);
}

static void pio_irq_rx_enable(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	data->irq_rx_enabled = true;
	pio_set_irqn_source_enabled(pio,
	                            config->interrupt_index,
	                            PIO_INTERRUPT_SOURCE_REL(pis_sm0_rx_fifo_not_empty, data->rx_sm),
	                            true);
}

static void pio_irq_rx_disable(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	data->irq_rx_enabled = false;
	pio_set_irqn_source_enabled(pio, config->interrupt_index, PIO_INTERRUPT_SOURCE_REL(pis_sm0_rx_fifo_not_empty, data->rx_sm), false);
}

static int pio_irq_rx_ready(const struct device *dev) {
	const struct pio_uart_config *config = dev->config;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	struct pio_uart_data *data = dev->data;

	return !pio_sm_is_rx_fifo_empty(pio, data->rx_sm);
}

static int pio_irq_is_pending(const struct device *dev) {
	struct pio_uart_data *data = dev->data;

	return
		(data->irq_tx_enabled && pio_irq_tx_ready(dev)) ||
		(data->irq_rx_enabled && pio_irq_rx_ready(dev));
}

static int pio_irq_update(const struct device *dev) {
	ARG_UNUSED(dev);

	return 1;
}

static void pio_irq_callback_set(const struct device *dev,
                                      uart_irq_callback_user_data_t cb,
                                      void *user_data) {
	struct pio_uart_data *data = dev->data;
	data->irq_cb = cb;
	data->user_data = user_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
#endif /* CONFIG_UART_ASYNC_API */

static int pio_uart_init(const struct device *dev)
{
	const struct pio_uart_config *config = dev->config;
	struct pio_uart_data *data = dev->data;
	float sm_clock_div;
	size_t tx_sm;
	size_t rx_sm;
	int retval;
	PIO pio;

	pio = pio_rpi_pico_get_pio(config->piodev);
	sm_clock_div = (float)clock_get_hz(clk_sys) / (CYCLES_PER_BIT * config->baudrate);

	retval = pio_rpi_pico_allocate_sm(config->piodev, &tx_sm);
	retval |= pio_rpi_pico_allocate_sm(config->piodev, &rx_sm);

	if (retval < 0) {
		return retval;
	}

	data->tx_sm = tx_sm;
	data->rx_sm = rx_sm;

	retval = pio_uart_tx_init(pio, tx_sm, config->tx_pin, sm_clock_div);
	if (retval < 0) {
		return retval;
	}

	retval = pio_uart_rx_init(pio, rx_sm, config->rx_pin, sm_clock_div);
	if (retval < 0) {
		return retval;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	config->irq_config_func(dev);
#endif

	return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void pio_uart_isr(const struct device *dev) {
	struct pio_uart_data *data = dev->data;

	if (data->irq_cb) {
		data->irq_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


static DEVICE_API(uart, pio_uart_driver_api) = {
	.poll_in = pio_uart_poll_in,
	.poll_out = pio_uart_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = pio_fifo_fill,
	.fifo_read = pio_fifo_read,
	.irq_tx_enable = pio_irq_tx_enable,
	.irq_tx_disable = pio_irq_tx_disable,
	.irq_tx_ready = pio_irq_tx_ready,
	.irq_tx_complete = pio_irq_tx_complete,
	.irq_rx_enable = pio_irq_rx_enable,
	.irq_rx_disable = pio_irq_rx_disable,
	.irq_rx_ready = pio_irq_rx_ready,
	// .irq_err_enable = pio_irq_err_enable,
	// .irq_err_disable = pio_irq_err_disable,
	.irq_is_pending = pio_irq_is_pending,
	.irq_update = pio_irq_update,
	.irq_callback_set = pio_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	// .callback_set = pio_callback_set,
	// .uart_tx = pio_callback_tx,
	// .uart_tx_abort = pio_tx_abort,
	// .uart_rx_enable = pio_rx_enable,
	// .uart_rx_buf_rsp = pio_rx_buf_rsp,
	// .uart_rx_disable = pi_rx_disable,
#endif /* CONFIG_UART_ASYNC_API */
};


#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
#define PIO_UART_INTERRUPT_INDEX(idx) DT_INST_PROP(idx, interrupt_index)

#define PIO_UART_IRQ_HANDLER_DEFINE(idx)					                       \
	static void pio_uart_irq_config_func_##idx(const struct device *dev)                           \
	{									                       \
		IRQ_CONNECT(DT_IRQN_BY_IDX(DT_INST_PARENT(idx), PIO_UART_INTERRUPT_INDEX(idx)),          \
			    DT_IRQ_BY_IDX(DT_INST_PARENT(idx), PIO_UART_INTERRUPT_INDEX(idx), priority), \
			    pio_uart_isr, DEVICE_DT_INST_GET(idx), 0);	                               \
		irq_enable(DT_IRQN_BY_IDX(DT_INST_PARENT(idx), PIO_UART_INTERRUPT_INDEX(idx)));				       \
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define PIO_UART_IRQ_HANDLER_FUNC(idx)					                              \
	IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (.irq_config_func = pio_uart_irq_config_func_##idx,))

#define PIO_UART_INIT(idx)									\
	PIO_UART_IRQ_HANDLER_DEFINE(idx)                                                        \
	PINCTRL_DT_INST_DEFINE(idx);								\
	static const struct pio_uart_config pio_uart##idx##_config = {				\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),					\
		.tx_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, tx_pins, 0),	\
		.rx_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(idx, default, 0, rx_pins, 0),	\
		.baudrate = DT_INST_PROP(idx, current_speed),					\
		.interrupt_index = PIO_UART_INTERRUPT_INDEX(idx),                               \
		PIO_UART_IRQ_HANDLER_FUNC(idx)                                                  \
	};											\
	static struct pio_uart_data pio_uart##idx##_data;					\
												\
	DEVICE_DT_INST_DEFINE(idx, pio_uart_init, NULL, &pio_uart##idx##_data,			\
			      &pio_uart##idx##_config, POST_KERNEL,				\
			      CONFIG_SERIAL_INIT_PRIORITY,					\
			      &pio_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_UART_INIT)
