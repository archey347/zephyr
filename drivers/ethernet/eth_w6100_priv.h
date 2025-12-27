/* W6100 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2024 Parth Sanepara
 * Author: Parth Sanepara <parthsanepara@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#ifndef ETH_W6100_PRIV_H_
#define ETH_W6100_PRIV_H_

// Common Register Locations
#define W6100_SYSR 0x2000  /* System Configuration Register */
#define W6100_SYCR0 0x2004 /* System Configuration Register 0 aka Soft Reset */
#define W6100_SIMR 0x2214  /* Socket Interrupt Mask Register */
#define W6100_RTR 0x4200   /* Retry Time Register */
#define W6100_SHAR 0x4120  /* Source MAC Address (len 6) */

// Common Register values
#define RTR_DEFAULT 2000   /* Expected default RTR value */
#define SIMR_S0 0x01 /* Socket Interrupt Mask Register for Socket 0 */

// Socket Register Locations
#define W6100_Sn(n, reg_offset) \
		(0x10000 + (n) * 0x40000 + (reg_offset))

// Use with W6100_Sn_REGISTER(n, <register offset>)
#define W6100_Sn_MR 0x0000 /* Socket Mode Register */
#define W6100_Sn_CR 0x0010 /* Socket Control Register */
#define W6100_Sn_TX_BSR 0x2000 /* Socket n TX Buffer Size Register */
#define W6100_Sn_RX_BSR 0x2200 /* Socket n RX Buffer Size Register */

// Socket Register Values
#define S_MR_MACRAW 0x04 /* Socket Mode Register for MACRAW mode */
#define S_MR_BRDB 0x40 /* Block broadcast udp packets */
#define S_CR_OPEN 0x01 /* Socket Control Register for OPEN command */
#define S_CR_CLOSE 0x10 /* Socket Control Register for CLOSE command */
#define S_CR_SEND 0x20 /* Socket Control Register for SEND command */
#define S_CR_RECV 0x40 /* Socket Control Register for RECV command */

/* Delay for PHY write/read operations (25.6 us) */
#define W6100_PHY_ACCESS_DELAY		26U
struct w6100_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec interrupt;
	struct gpio_dt_spec reset;
	int32_t timeout;
	struct net_eth_mac_config mac_cfg;
};

struct w6100_runtime {
	struct net_if *iface;

	K_KERNEL_STACK_MEMBER(thread_stack,
			      CONFIG_ETH_W6100_RX_THREAD_STACK_SIZE);
	struct k_thread thread;
	uint8_t mac_addr[6];
	struct gpio_callback gpio_cb;
	struct k_sem tx_sem;
	struct k_sem int_sem;
	bool link_up;
	uint8_t buf[NET_ETH_MAX_FRAME_SIZE];
};
#endif /*ETH_W6100_PRIV_H_*/
