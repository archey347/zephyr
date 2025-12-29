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
#define W6100_SYCR1 0x2005 /* System Configuration Register 1 */

#define W6100_IR 0x2100    /* Interrupt Register */
#define W6100_IMR 0x2104   /* Interrupt Mask Register */
#define W6100_IRCLR 0x2108   /* Interrupt Register Clear */

#define W6100_SLIR 0x2102   /* Socket-less Interrupt Register */
#define W6100_SLIMR 0x2124   /* Socket-less Interrupt Mask Register */
#define W6100_SLIRCLR 0x2128 /* Socket-less Interrupt Register Clear */

#define W6100_SIR 0x2101   /* Socket Interrupt Register */
#define W6100_SIMR 0x2114   /* Socket Interrupt Mask Register */

#define W6100_PHYSR 0x3000 /* PHY Status Register */
#define W6100_RTR 0x4200   /* Retry Time Register */
#define W6100_SHAR 0x4120  /* Source MAC Address (len 6) */
#define W6100_CHPLCKR 0x41F4 /* Chip Lock Register */

// Common Register values
#define RTR_DEFAULT 2000   /* Expected default RTR value */
#define SIMR_S0 0x01 /* Socket Interrupt Mask Value for Socket 0 */
#define SYCR0_SOFT_RESET 0x80 /* Soft reset value */
#define CHPLCKR_UNLOCK 0xCE /* Chip Lock Register Unlock Value */

// Socket Register Locations
#define W6100_Sn(n, reg_offset) \
		(0x10000 + (n) * 0x40000 + (reg_offset))

// Use with W6100_Sn(n, <register offset>)
#define W6100_Sn_MR 0x0000 /* Socket Mode Register */
#define W6100_Sn_CR 0x0010 /* Socket Control Register */
#define W6100_S_IR 0x0020 /* Socket Interrupt Register */
#define W6100_S_TX_WR 0x020C /* Socket n TX Write Pointer Register */
#define W6100_S_IMR 0x0024 /* Socket Interrupt Mask Register */
#define W6100_S_IRCLR 0x0028 /* Socket Interrupt Register Clear */
#define W6100_Sn_TX_BSR 0x2000 /* Socket n TX Buffer Size Register */
#define W6100_Sn_RX_BSR 0x2200 /* Socket n RX Buffer Size Register */
#define W6100_S_RX_RSR 0x0224 /* Socket n RX Receive Size Register */
#define W6100_S_RX_RD 0x0228 /* Socket n RX Read Pointer Register */

// Socket Register Values
#define S_MR_MACRAW 0x07 /* Socket Mode Register for MACRAW mode */
#define S_MR_BRDB 0x40 /* Block broadcast udp packets */
#define S_CR_OPEN 0x01 /* Socket Control Register for OPEN command */
#define S_CR_CLOSE 0x10 /* Socket Control Register for CLOSE command */
#define S_CR_SEND 0x20 /* Socket Control Register for SEND command */
#define S_CR_RECV 0x40 /* Socket Control Register for RECV command */
#define S_IR_SENDOK 0x10 /* Socket Interrupt Register for SENDOK event */
#define S_IR_RECV 0x04 /* Socket Interrupt Register for RECV event */
#define S_IMR_SENDOK 0x10
#define S_IMR_RECV 0x04

// Socket RX/TX Memory Offsets (use with S6100_Sn(n, start))
#define W6100_S_TX_MEM_START 0x10000
#define W6100_S_RX_MEM_START 0x20000
#define W6100_S_TX_MEM_SIZE 0x04000
#define W6100_S_RX_MEM_SIZE 0x04000



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
