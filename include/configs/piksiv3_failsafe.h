/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __CONFIG_PIKSIV3_FAILSAFE_H
#define __CONFIG_PIKSIV3_FAILSAFE_H

#define CONFIG_SYS_NO_FLASH

#define CONFIG_ZYNQ_SDHCI0
#define CONFIG_ZYNQ_USB

/* ATAG support */
#define CONFIG_CMDLINE_TAG

/* CRC verfication support */
#define CONFIG_HASH_VERIFY

 /* Factory data */
#define CONFIG_FACTORY_DATA
#define CONFIG_FACTORY_DATA_OFFSET 0x00040000U
#define CONFIG_FACTORY_DATA_FALLBACK
#define CONFIG_ZYNQ_GEM_FACTORY_ADDR

 /* Image table */
#define CONFIG_IMAGE_TABLE_BOOT
#define CONFIG_IMAGE_SET_OFFSET_FAILSAFE_A 0x00100000U
#define CONFIG_IMAGE_SET_OFFSET_FAILSAFE_B 0x00140000U
#define CONFIG_IMAGE_SET_OFFSET_STANDARD_A 0x00180000U
#define CONFIG_IMAGE_SET_OFFSET_STANDARD_B 0x001C0000U
#define CONFIG_IMAGE_SET_OFFSETS          \
    {CONFIG_IMAGE_SET_OFFSET_FAILSAFE_A,  \
     CONFIG_IMAGE_SET_OFFSET_FAILSAFE_B,  \
     CONFIG_IMAGE_SET_OFFSET_STANDARD_A,  \
     CONFIG_IMAGE_SET_OFFSET_STANDARD_B}

#define CONFIG_DISABLE_ZYNQ_DEBUG

#define CONFIG_PREBOOT                                 \
  "echo Erasing flash...; "                            \
  "if sf probe                                    && " \
  "   echo Erasing standard partition A header... && " \
  "   sf erase 0x0180000 0x0040000                && " \
  "   echo Erasing standard partition B header... && " \
  "   sf erase 0x01C0000 0x0040000                && " \
  "   echo Erasing standard SPL A partition...    && " \
  "   sf erase 0x0280000 0x0040000                && " \
  "   echo Erasing standard SPL B partition...    && " \
  "   sf erase 0x02C0000 0x0040000                && " \
  "   echo Erasing standard A image partition...  && " \
  "   sf erase 0x0300000 0x1c00000                && " \
  "   echo Erasing standard B image partition...  && " \
  "   sf erase 0x2000000 0x1c00000; "                  \
  "then "                                              \
  "   echo Enabling JTAG...; "                         \
  "   enable_jtag; "                                   \
  "else "                                              \
  "   echo Flash erase failed, resetting in 5s...; "   \
  "   sleep 1; "                                       \
  "   echo Flash erase failed, resetting in 4s...; "   \
  "   sleep 1; "                                       \
  "   echo Flash erase failed, resetting in 3s...; "   \
  "   sleep 1; "                                       \
  "   echo Flash erase failed, resetting in 2s...; "   \
  "   sleep 1; "                                       \
  "   echo Flash erase failed, resetting in 1s...; "   \
  "   sleep 1; "                                       \
  "   reset; "                                         \
  "fi"                                                 \

/* CPU clock */
#ifndef CONFIG_CPU_FREQ_HZ
# define CONFIG_CPU_FREQ_HZ 800000000
#endif

/* Cache options */
#define CONFIG_CMD_CACHE
#define CONFIG_SYS_CACHELINE_SIZE 32

#define CONFIG_SYS_L2CACHE_OFF
#ifndef CONFIG_SYS_L2CACHE_OFF
# define CONFIG_SYS_L2_PL310
# define CONFIG_SYS_PL310_BASE    0xf8f02000
#endif

#define ZYNQ_SCUTIMER_BASEADDR    0xF8F00600
#define CONFIG_SYS_TIMERBASE    ZYNQ_SCUTIMER_BASEADDR
#define CONFIG_SYS_TIMER_COUNTS_DOWN
#define CONFIG_SYS_TIMER_COUNTER  (CONFIG_SYS_TIMERBASE + 0x4)

/* Serial drivers */
#define CONFIG_BAUDRATE   115200
/* The following table includes the supported baudrates */
#define CONFIG_SYS_BAUDRATE_TABLE   {CONFIG_BAUDRATE}

#define CONFIG_ARM_DCC
#define CONFIG_ZYNQ_SERIAL

/* Ethernet driver */
#if defined(CONFIG_ZYNQ_GEM)
# define CONFIG_MII
# define CONFIG_SYS_FAULT_ECHO_LINK_DOWN
# define CONFIG_PHY_MARVELL
# define CONFIG_PHY_XILINX
# define CONFIG_SYS_ENET
# define CONFIG_BOOTP_BOOTPATH
# define CONFIG_BOOTP_GATEWAY
# define CONFIG_BOOTP_HOSTNAME
# define CONFIG_BOOTP_MAY_FAIL
#endif

/* SPI */
#ifdef CONFIG_ZYNQ_SPI
# define CONFIG_CMD_SF
#endif

/* QSPI */
#ifdef CONFIG_ZYNQ_QSPI
# define CONFIG_SF_DEFAULT_SPEED 40000000
# define CONFIG_CMD_SF
# define CONFIG_SF_DUAL_FLASH
#endif

/* MMC */
#if defined(CONFIG_ZYNQ_SDHCI0) || defined(CONFIG_ZYNQ_SDHCI1)
# define CONFIG_MMC
# define CONFIG_GENERIC_MMC
# define CONFIG_SDHCI
# define CONFIG_ZYNQ_SDHCI
# define CONFIG_CMD_MMC
# define CONFIG_ZYNQ_SDHCI_MAX_FREQ 52000000
#endif

#ifdef CONFIG_ZYNQ_USB
# define CONFIG_USB_EHCI
# define CONFIG_CMD_USB
# define CONFIG_USB_STORAGE
# define CONFIG_USB_EHCI_ZYNQ
# define CONFIG_USB_ULPI_VIEWPORT
# define CONFIG_USB_ULPI
# define CONFIG_EHCI_IS_TDI
# define CONFIG_USB_MAX_CONTROLLER_COUNT  2

# define CONFIG_CI_UDC           /* ChipIdea CI13xxx UDC */
# define CONFIG_USB_GADGET
# define CONFIG_USB_GADGET_DUALSPEED
# define CONFIG_USB_GADGET_DOWNLOAD
# define CONFIG_SYS_DFU_DATA_BUF_SIZE 0x600000
# define DFU_DEFAULT_POLL_TIMEOUT 300
# define CONFIG_USB_FUNCTION_DFU
# define CONFIG_DFU_RAM
# define CONFIG_USB_GADGET_VBUS_DRAW  2
# define CONFIG_G_DNL_VENDOR_NUM  0x03FD
# define CONFIG_G_DNL_PRODUCT_NUM 0x0300
# define CONFIG_G_DNL_MANUFACTURER  "Xilinx"
# define CONFIG_USB_GADGET
# define CONFIG_USB_CABLE_CHECK
# define CONFIG_CMD_DFU
# define CONFIG_CMD_THOR_DOWNLOAD
# define CONFIG_THOR_RESET_OFF
# define CONFIG_USB_FUNCTION_THOR
#endif

#if defined(CONFIG_ZYNQ_SDHCI) || defined(CONFIG_ZYNQ_USB)
# define CONFIG_SUPPORT_VFAT
# define CONFIG_CMD_FAT
# define CONFIG_CMD_EXT2
# define CONFIG_FAT_WRITE
# define CONFIG_DOS_PARTITION
# define CONFIG_CMD_EXT4
# define CONFIG_CMD_EXT4_WRITE
# define CONFIG_CMD_FS_GENERIC
#endif

/* Environment */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE     (256 << 10)

/* Default environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
  "autoload=no\0" \
  "net_disable_gigabit=" \
    "mdio write 9 0; " \
    "mdio write 0 0; " \
    "mdio write 0 1000; " \
    "sleep 1;\0"

/* Default environment */
#define CONFIG_BOOTCOMMAND ""

#define CONFIG_BOOTDELAY    -1 /* -1 to Disable autoboot */
#define CONFIG_SYS_LOAD_ADDR    0 /* default? */

/* Miscellaneous configurable options */
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SYS_LONGHELP
#define CONFIG_CLOCKS
#define CONFIG_CMD_CLK
#define CONFIG_SYS_MAXARGS    32 /* max number of command args */
#define CONFIG_SYS_CBSIZE   2048 /* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE   (CONFIG_SYS_CBSIZE + \
          sizeof(CONFIG_SYS_PROMPT) + 16)

/* Physical Memory map */
#define CONFIG_SYS_TEXT_BASE    0x4000000
#define CONFIG_SYS_UBOOT_START  CONFIG_SYS_TEXT_BASE

#define CONFIG_NR_DRAM_BANKS    1
#define CONFIG_SYS_SDRAM_BASE   0

#define CONFIG_SYS_MEMTEST_START  CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END    (CONFIG_SYS_SDRAM_BASE + 0x1000)

#define CONFIG_SYS_MALLOC_LEN   0xC00000

#define CONFIG_SYS_INIT_RAM_ADDR  0xFFFF0000
#define CONFIG_SYS_INIT_RAM_SIZE  0x1000
#define CONFIG_SYS_INIT_SP_ADDR   (CONFIG_SYS_INIT_RAM_ADDR + \
          CONFIG_SYS_INIT_RAM_SIZE - \
          GENERATED_GBL_DATA_SIZE)

/* Enable the PL to be downloaded */
#define CONFIG_FPGA
#define CONFIG_FPGA_XILINX
#define CONFIG_FPGA_ZYNQPL
#define CONFIG_CMD_FPGA_LOADMK
#define CONFIG_CMD_FPGA_LOADP
#define CONFIG_CMD_FPGA_LOADBP
#define CONFIG_CMD_FPGA_LOADFS

/* Open Firmware flat tree */
#define CONFIG_OF_LIBFDT

/* FIT support */
#define CONFIG_IMAGE_FORMAT_LEGACY /* enable also legacy image format */

/* FDT support */
#define CONFIG_DISPLAY_BOARDINFO_LATE

/* Extend size of kernel image for uncompression */
#define CONFIG_SYS_BOOTM_LEN  (60 * 1024 * 1024)

/* Boot FreeBSD/vxWorks from an ELF image */
#define CONFIG_SYS_MMC_MAX_DEVICE 1

#define CONFIG_SYS_LDSCRIPT  "arch/arm/mach-zynq/u-boot.lds"

/* Commands */
#ifdef CONFIG_SYS_ENET
# define CONFIG_CMD_PING
# define CONFIG_CMD_DHCP
# define CONFIG_CMD_MII
# define CONFIG_CMD_TFTPPUT
#else
# undef CONFIG_CMD_NET
# undef CONFIG_CMD_NFS
#endif

#if defined(CONFIG_CMD_ZYNQ_RSA)
# ifndef CONFIG_RSA
#  define CONFIG_RSA
# endif
#define CONFIG_SHA256
#define CONFIG_CMD_ZYNQ_AES
#endif

#define CONFIG_CMD_BOOTZ
#undef CONFIG_BOOTM_NETBSD

#define CONFIG_CMD_IMAGE_SET
#define CONFIG_CMD_ENABLE_JTAG
#define CONFIG_CMD_DISABLE_JTAG

#define CONFIG_SYS_HZ     1000

/* For development/debugging */
#ifdef DEBUG
# define CONFIG_CMD_REGINFO
# define CONFIG_PANIC_HANG
#endif

/* SPL part */
#define CONFIG_CMD_SPL
#define CONFIG_SPL_FRAMEWORK
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_BOARD_INIT

#define CONFIG_SPL_LDSCRIPT "arch/arm/mach-zynq/u-boot-spl.lds"

/* MMC support */
#ifdef CONFIG_ZYNQ_SDHCI0
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 0x300 /* address 0x60000 */
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS      0x200 /* 256 KB */
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION     1
#define CONFIG_SPL_LIBDISK_SUPPORT
#define CONFIG_SPL_FAT_SUPPORT
#ifdef CONFIG_OF_SEPARATE
# define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME     "u-boot-dtb.img"
#else
# define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME     "u-boot.img"
#endif
#endif

/* Disable dcache for SPL just for sure */
#ifdef CONFIG_SPL_BUILD
#define CONFIG_SYS_DCACHE_OFF
#undef CONFIG_FPGA
#endif

/* Address in RAM where the parameters must be copied by SPL. */
#define CONFIG_SYS_SPL_ARGS_ADDR  0x10000000

#define CONFIG_SPL_FS_LOAD_ARGS_NAME    "system.dtb"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME    "uImage"

/* Not using MMC raw mode - just for compilation purpose */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR 0
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS  0
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR 0

/* qspi mode is working fine */
#ifdef CONFIG_ZYNQ_QSPI
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SYS_SPI_U_BOOT_OFFS  0x100000
#define CONFIG_SYS_SPI_ARGS_OFFS  0x200000
#define CONFIG_SYS_SPI_ARGS_SIZE  0x80000
#define CONFIG_SYS_SPI_KERNEL_OFFS  (CONFIG_SYS_SPI_ARGS_OFFS + \
          CONFIG_SYS_SPI_ARGS_SIZE)
#endif

#ifdef DEBUG
#define CONFIG_SPL_RAM_DEVICE
#define CONFIG_SPL_NET_SUPPORT
#define CONFIG_SPL_ETH_SUPPORT
#define CONFIG_SPL_ENV_SUPPORT
#define CONFIG_SPL_ETH_DEVICE "Gem.e000b000"
#endif

/* Use image table */
#define CONFIG_SPL_BOARD_LOAD_IMAGE

/* for booting directly linux */
#define CONFIG_SPL_OS_BOOT

/* SPL code */
/* Workaround for TPL icache issue: Do not place code in lower 40kB */
#define CONFIG_SPL_TEXT_BASE      0x0000a000
#define CONFIG_SPL_MAX_SIZE       0x00026000

/* The highest 64k OCM address */
#define OCM_HIGH_ADDR 0xffff0000

/* On the top of OCM space */
#define CONFIG_SYS_SPL_MALLOC_START OCM_HIGH_ADDR
#define CONFIG_SYS_SPL_MALLOC_SIZE  0x2000

/*
 * SPL stack position - and stack goes down
 * 0xfffffe00 is used for putting wfi loop.
 * Set it up as limit for now.
 */
#define CONFIG_SPL_STACK  0xfffffe00

/* BSS setup */
#define CONFIG_SPL_BSS_START_ADDR 0x3000000
#define CONFIG_SPL_BSS_MAX_SIZE   0x100000

#endif /* __CONFIG_PIKSIV3_FAILSAFE_H */
