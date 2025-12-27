/* W6100 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2024 Parth Sanepara
 * Author: Parth Sanepara <parthsanepara@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT	wiznet_w6100

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_w6100, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>

#include "eth.h"
#include "eth_w6100_priv.h"

#define W6100_SPI_BLOCK_SELECT(addr)	(((addr) >> 16) & 0x1f)
#define W6100_SPI_READ_CONTROL(addr)	(W6100_SPI_BLOCK_SELECT(addr) << 3)
#define W6100_SPI_WRITE_CONTROL(addr)   \
	((W6100_SPI_BLOCK_SELECT(addr) << 3) | BIT(2))

static int w6100_spi_read(const struct device *dev, uint32_t addr,
				  uint8_t *data, size_t len)
{
		const struct w6100_config *cfg = dev->config;
		int ret;

		uint8_t cmd[3] = {
			addr >> 8,
			addr,
			W6100_SPI_READ_CONTROL(addr)
		};
		const struct spi_buf tx_buf = {
			.buf = cmd,
			.len = ARRAY_SIZE(cmd),
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1,
		};
		/* skip the default dummy 0x010203 */
		const struct spi_buf rx_buf[2] = {
			{
				.buf = NULL,
				.len = 3
			},
			{
				.buf = data,
				.len = len
			},
		};
		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = ARRAY_SIZE(rx_buf),
		};

		ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
		return ret;
}

static int w6100_spi_write(const struct device *dev, uint32_t addr,
				   uint8_t *data, size_t len)
{
		const struct w6100_config *cfg = dev->config;
		int ret;
		uint8_t cmd[3] = {
			addr >> 8,
			addr,
			W6100_SPI_WRITE_CONTROL(addr),
		};
		const struct spi_buf tx_buf[2] = {
			{
				.buf = cmd,
				.len = ARRAY_SIZE(cmd),
			},
			{
				.buf = data,
				.len = len,
			},
		};
		const struct spi_buf_set tx = {
			.buffers = tx_buf,
			.count = ARRAY_SIZE(tx_buf),
		};

		ret = spi_write_dt(&cfg->spi, &tx);

		return ret;
}

static void w6100_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

}

static void w6100_iface_init(struct net_if *iface)
{
}

static int w6100_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct w6100_runtime *ctx = dev->data;

	return 0;
}

static enum ethernet_hw_caps w6100_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
		| ETHERNET_PROMISC_MODE
#endif
	;
}

static int w6100_set_config(const struct device *dev,
			    enum ethernet_config_type type,
			    const struct ethernet_config *config)
{
}

static int w6100_command(const struct device *dev, uint8_t cmd)
{
	uint8_t reg;
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(100));

	w6100_spi_write(dev, W6100_Sn(0, W6100_Sn_CR), &cmd, 1);
	while (true) {
		w6100_spi_read(dev, W6100_Sn(0, W6100_Sn_CR), &reg, 1);
		if (!reg) {
			break;
		}
		if (sys_timepoint_expired(end)) {
			return -EIO;
		}
		k_busy_wait(W6100_PHY_ACCESS_DELAY);
	}
	return 0;
}

static int w6100_hw_start(const struct device *dev)
{
    uint8_t mode = S_MR_MACRAW | BIT(S_MR_BRDB);
	uint8_t mask = SIMR_S0;

	/* configure Socket 0 with MACRAW mode and MAC filtering enabled */
	w6100_spi_write(dev, W6100_Sn(0, W6100_Sn_MR), &mode, 1);
	w6100_command(dev, S_CR_OPEN);

	/* enable interrupt */
	w6100_spi_write(dev, W6100_SIMR, &mask, 1);

    return 0;
}

static int w6100_hw_stop(const struct device *dev)
{
}

static const struct ethernet_api w6100_api_funcs = {
	.iface_api.init = w6100_iface_init,
	.get_capabilities = w6100_get_capabilities,
	.set_config = w6100_set_config,
	.start = w6100_hw_start,
	.stop = w6100_hw_stop,
	.send = w6100_tx,
};

static void w6100_set_macaddr(const struct device *dev)
{
	struct w6100_runtime *ctx = dev->data;

	w6100_spi_write(dev, W6100_SHAR, ctx->mac_addr, sizeof(ctx->mac_addr));
}

static void w6100_gpio_callback(const struct device *dev,
				struct gpio_callback *cb,
				uint32_t pins)
{
	struct w6100_runtime *ctx =
		CONTAINER_OF(cb, struct w6100_runtime, gpio_cb);

	k_sem_give(&ctx->int_sem);
}

static void w6100_memory_configure(const struct device *dev)
{
	int i;
	uint8_t mem = 0x10;

	/* Configure RX & TX memory to 16K for socket 0 */
	w6100_spi_write(dev, W6100_Sn(0, W6100_Sn_RX_BSR), &mem, 1);
	w6100_spi_write(dev, W6100_Sn(0, W6100_Sn_TX_BSR), &mem, 1);

	/* Remove memory from all other sockets */
	mem = 0;
	for (i = 1; i < 8; i++) {
		w6100_spi_write(dev, W6100_Sn(i, W6100_Sn_RX_BSR), &mem, 1);
		w6100_spi_write(dev, W6100_Sn(i, W6100_Sn_TX_BSR), &mem, 1);
	}
}

static int w6100_init(const struct device *dev)
{
    int err;
    uint8_t rtr[2];
    const struct w6100_config *config = dev->config;
    struct w6100_runtime *ctx         = dev->data;

    ctx->link_up = false;

    if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI master port %s not ready", config->spi.bus->name);
		return -EINVAL;
	}

	if (!gpio_is_ready_dt(&config->interrupt)) {
		LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
		return -EINVAL;
	}

	err = gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT);
	if (err < 0) {
		LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
		return err;
	}

	gpio_init_callback(&(ctx->gpio_cb), w6100_gpio_callback,
			   BIT(config->interrupt.pin));
	err = gpio_add_callback(config->interrupt.port, &(ctx->gpio_cb));
	if (err < 0) {
		LOG_ERR("Unable to add GPIO callback %u", config->interrupt.pin);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&config->interrupt,
					      GPIO_INT_EDGE_FALLING);
	if (err < 0) {
		LOG_ERR("Unable to enable GPIO INT %u", config->interrupt.pin);
		return err;
	}

	if (config->reset.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset)) {
			LOG_ERR("GPIO port %s not ready", config->reset.port->name);
			return -EINVAL;
		}

		err = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			LOG_ERR("Unable to configure GPIO pin %u", config->reset.pin);
			return err;
		}

		/* See Section 8.4.1 of the W6100 datasheet
		 * Trst = 580us
		 * Tsta = 60.3ms
		 */
		gpio_pin_set_dt(&config->reset, 1);
		k_usleep(580);
		gpio_pin_set_dt(&config->reset, 0);
		k_msleep(65);
	}

	//err = w6100_soft_reset(dev);
	// W6100 has soft reset (see SYCR0)
	// but I couldn't figure out the status lock stuff

	// Disable interrupts on all sockets
	uint8_t mask = 0;
	err = w6100_spi_write(dev, W6100_SIMR, &mask, 1);
	if (err != 0) {
		LOG_ERR("Failed to disable socket interrupts");
		return err;
	}

	(void)net_eth_mac_load(&config->mac_cfg, ctx->mac_addr);
	w6100_set_macaddr(dev);
	w6100_memory_configure(dev);

	/* check retry time value */
	w6100_spi_read(dev, W6100_RTR, rtr, 2);
	if (sys_get_be16(rtr) != RTR_DEFAULT) {
		LOG_ERR("Unable to read RTR register");
		return -ENODEV;
	}

	k_thread_create(&ctx->thread, ctx->thread_stack,
			CONFIG_ETH_W6100_RX_THREAD_STACK_SIZE,
			w6100_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_W6100_RX_THREAD_PRIO),
			0, K_NO_WAIT);
	k_thread_name_set(&ctx->thread, "eth_w6100");

    LOG_INF("W6100 Initialized");

    return 0;
}

static struct w6100_runtime w6100_0_runtime = {
	.tx_sem = Z_SEM_INITIALIZER(w6100_0_runtime.tx_sem,
					1,  UINT_MAX),
	.int_sem  = Z_SEM_INITIALIZER(w6100_0_runtime.int_sem,
				      0, UINT_MAX),
};

static const struct w6100_config w6100_0_config = {
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8), 0),
	.interrupt = GPIO_DT_SPEC_INST_GET(0, int_gpios),
	.reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, { 0 }),
	.timeout = CONFIG_ETH_W6100_TIMEOUT,
	.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(0),
};

ETH_NET_DEVICE_DT_INST_DEFINE(0,
		    w6100_init, NULL,
		    &w6100_0_runtime, &w6100_0_config,
		    CONFIG_ETH_INIT_PRIORITY, &w6100_api_funcs, NET_ETH_MTU);
