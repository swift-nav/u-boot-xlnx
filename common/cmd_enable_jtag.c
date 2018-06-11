/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <common.h>
#include <command.h>

#include <asm/io.h>
#include <asm/arch/clk.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/hardware.h>

#define DAP_ENABLE_MASK 0x7F
#define JTAG_CHAIN_DISABLE 0x00800000

static void write_debug_enable()
{
  // See: https://www.xilinx.com/support/answers/64275.html

  unsigned int control = readl(&devcfg_base->ctrl);

  control |=  DAP_ENABLE_MASK;
  control &= ~JTAG_CHAIN_DISABLE;

  writel(control, &devcfg_base->ctrl);
}

int do_enable_jtag(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
  write_debug_enable(); 
  return CMD_RET_SUCCESS;
}
/***************************************************/

U_BOOT_CMD(
  enable_jtag, CONFIG_SYS_MAXARGS, 0, do_enable_jtag,
  "Re-enables JTAG",
  "Writes control register which re-enables JTAG."
);
