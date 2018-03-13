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

#include <common.h>
#include <command.h>
#include <spi.h>
#include <spi_flash.h>
#include <image_table.h>

#define IMAGE_ALIGN 16U

static int packed_image_set_verify_all(uint32_t packed_image_set_addr)
{
  const image_set_t *image_set = (const image_set_t *)packed_image_set_addr;
  const uint8_t *image_set_data = (const uint8_t *)packed_image_set_addr;

  /* Verify image set header */
  if (image_set_verify(image_set) != 0) {
    return -1;
  }

  /* Verify image data */
  int i;
  for (i=0; i<IMAGE_SET_DESCRIPTORS_COUNT; i++) {
    const image_descriptor_t *d = &image_set->descriptors[i];
    if (image_descriptor_type_get(d) != IMAGE_TYPE_INVALID) {

      const uint8_t *image_data =
          &image_set_data[image_descriptor_data_offset_get(d)];
      uint32_t image_data_size = image_descriptor_data_size_get(d);

      uint32_t computed_data_crc;
      image_descriptor_data_crc_init(&computed_data_crc);
      image_descriptor_data_crc_continue(&computed_data_crc, image_data,
                                         image_data_size);

      if (image_descriptor_data_crc_get(d) != computed_data_crc) {
        return -1;
      }
    }
  }

  return 0;
}

static int packed_image_set_write_all(uint32_t packed_image_set_addr,
                                      uint32_t set_offset, uint32_t spl_offset,
                                      uint32_t std_offset)
{
  /* Probe flash */
  struct spi_flash *flash;
  flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
                          CONFIG_SF_DEFAULT_CS,
                          CONFIG_SF_DEFAULT_SPEED,
                          CONFIG_SF_DEFAULT_MODE);
  if (!flash) {
    puts("SPI probe failed\n");
    return -ENODEV;
  }

  const image_set_t *image_set = (const image_set_t *)packed_image_set_addr;
  const uint8_t *image_set_data = (const uint8_t *)packed_image_set_addr;

  /* Initialize target image set */
  image_set_t target_image_set;
  memcpy(&target_image_set, image_set, sizeof(image_set_t));

  /* Write each image to flash */
  uint32_t std_sub_offset = 0;
  int i;
  for (i=0; i<IMAGE_SET_DESCRIPTORS_COUNT; i++) {
    const image_descriptor_t *d = &image_set->descriptors[i];

    uint32_t type = image_descriptor_type_get(d);
    if (type == IMAGE_TYPE_INVALID) {
      continue;
    }

    const uint8_t *image_data =
        &image_set_data[image_descriptor_data_offset_get(d)];
    uint32_t image_data_size = image_descriptor_data_size_get(d);

    /* Determine flash offset */
    uint32_t flash_offset;
    if (type == IMAGE_TYPE_UBOOT_SPL) {
      flash_offset = spl_offset;
    } else {
      flash_offset = std_offset + std_sub_offset;
      std_sub_offset += image_data_size;
      if ((std_sub_offset % IMAGE_ALIGN) != 0) {
        std_sub_offset += IMAGE_ALIGN - (std_sub_offset % IMAGE_ALIGN);
      }
    }

    /* Write to flash */
    if (spi_flash_write(flash, flash_offset,
                        image_data_size, (void *)image_data) != 0) {
      puts("SPI write failed\n");
      return -1;
    }

    /* Update target image set descriptor */
    image_descriptor_data_offset_set(&target_image_set.descriptors[i],
                                     flash_offset);
  }

  /* Update target image set CRC */
  image_set_finalize(&target_image_set);

  /* Write target image set header to flash */
  if (spi_flash_write(flash, set_offset,
                      sizeof(image_set_t), (void *)&target_image_set) != 0) {
    puts("SPI write failed\n");
    return -1;
  }

  return 0;
}

static int image_set_verify_all(uint32_t set_offset, uint32_t ram_addr)
{
  /* Probe flash */
  struct spi_flash *flash;
  flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
                          CONFIG_SF_DEFAULT_CS,
                          CONFIG_SF_DEFAULT_SPEED,
                          CONFIG_SF_DEFAULT_MODE);
  if (!flash) {
    puts("SPI probe failed\n");
    return -ENODEV;
  }

  /* Load image set header from flash into RAM */
  image_set_t image_set;
  if (spi_flash_read(flash, set_offset, sizeof(image_set_t),
                     (void *)&image_set) != 0) {
    puts("Failed to read image set\n");
    return -1;
  }

  /* Verify image set header */
  if (image_set_verify(&image_set) != 0) {
    return -1;
  }

  /* Verify image data */
  int i;
  for (i=0; i<IMAGE_SET_DESCRIPTORS_COUNT; i++) {
    const image_descriptor_t *d = &image_set.descriptors[i];
    if (image_descriptor_type_get(d) != IMAGE_TYPE_INVALID) {

      /* Load image data from flash into RAM */
      uint8_t *image_data = (uint8_t *)ram_addr;
      uint32_t image_data_offset = image_descriptor_data_offset_get(d);
      uint32_t image_data_size = image_descriptor_data_size_get(d);

      if (spi_flash_read(flash, image_data_offset, image_data_size,
                         (void *)image_data) != 0) {
        puts("Failed to read image data\n");
        return -1;
      }

      uint32_t computed_data_crc;
      image_descriptor_data_crc_init(&computed_data_crc);
      image_descriptor_data_crc_continue(&computed_data_crc, image_data,
                                         image_data_size);

      if (image_descriptor_data_crc_get(d) != computed_data_crc) {
        return -1;
      }
    }
  }

  return 0;
}

static void print_image_type(uint32_t image_type) {
  puts("\t\ttype: ");
  switch(image_type) {
    case IMAGE_TYPE_UNKNOWN:
      puts("IMAGE_TYPE_UNKNOWN");
      break;
    case IMAGE_TYPE_LOADER:
      puts("IMAGE_TYPE_LOADER");
      break;
    case IMAGE_TYPE_UBOOT_SPL:
      puts("IMAGE_TYPE_UBOOT_SPL");
      break;
    case IMAGE_TYPE_UBOOT:
      puts("IMAGE_TYPE_UBOOT");
      break;
    case IMAGE_TYPE_LINUX:
      puts("IMAGE_TYPE_LINUX");
      break;
    case IMAGE_TYPE_FPGA:
      puts("IMAGE_TYPE_FPGA");
      break;
    default:
      puts("IMAGE_TYPE_INVALID");
  }
  puts("\n");
}

static void print_set_hw(uint32_t set_hw) {
  puts("\thw: ");
  switch(set_hw) {
    case IMAGE_HARDWARE_UNKNOWN:
      puts("IMAGE_HARDWARE_UNKNOWN");
      break;
    case IMAGE_HARDWARE_V3_MICROZED:
      puts("IMAGE_HARDWARE_V3_MICROZED");
      break;
    case IMAGE_HARDWARE_V3_EVT1:
      puts("IMAGE_HARDWARE_V3_EVT1");
      break;
    case IMAGE_HARDWARE_V3_EVT2:
      puts("IMAGE_HARDWARE_V3_EVT2");
      break;
    case IMAGE_HARDWARE_V3_PROD:
      puts("IMAGE_HARDWARE_V3_PROD");
      break;
    default:
      puts("IMAGE_HARDWARE_INVALID");
  }
  puts("\n");
}

static void sanitize_image_str(char * str_buf, int len) {
  str_buf[len - 1] = 0;
  for (int i =0; i < len; i++) {
    if (str_buf[i] == IMAGE_SET_RESERVED_BYTE) {
      str_buf[i] = 0;
      break;
    }
  }
}

static int image_set_info(uint32_t set_offset, uint32_t ram_addr)
{
  /* Probe flash */
  struct spi_flash *flash;
  flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
                          CONFIG_SF_DEFAULT_CS,
                          CONFIG_SF_DEFAULT_SPEED,
                          CONFIG_SF_DEFAULT_MODE);
  if (!flash) {
    puts("SPI probe failed\n");
    return -ENODEV;
  }

  /* Load image set header from flash into RAM */
  image_set_t image_set;
  if (spi_flash_read(flash, set_offset, sizeof(image_set_t),
                     (void *)&image_set) != 0) {
    puts("Failed to read image set\n");
    return -1;
  }

  const int BUF_LEN = 64;
  char str_buf[BUF_LEN];
  puts("Image Set:\n");
  image_set_name_get(&image_set, (uint8_t *)str_buf);
  sanitize_image_str(str_buf, BUF_LEN);
  printf("\tname: %s\n", str_buf);
  printf("\tseq_num: %d\n", image_set_seq_num_get(&image_set));
  print_set_hw(image_set_hardware_get(&image_set));

  /* Verify image data */
  int i;
  for (i=0; i<IMAGE_SET_DESCRIPTORS_COUNT; i++) {
    const image_descriptor_t *d = &image_set.descriptors[i];
    if (image_descriptor_type_get(d) != IMAGE_TYPE_INVALID) {
      puts("\tImage:\n");
      image_descriptor_name_get(d, (uint8_t *)str_buf);
      sanitize_image_str(str_buf, BUF_LEN);
      printf("\t\tname: %s\n", str_buf);
      printf("\t\tsize: %d\n", image_descriptor_data_size_get(d));
      print_image_type(image_descriptor_type_get(d));
    }
  }

  return 0;
}

int do_image_set(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
  if (argc < 2) {
    return CMD_RET_USAGE;
  }

  if (strcmp(argv[1], "write") == 0) {

    if (argc != 6) {
      return CMD_RET_USAGE;
    }

    uint32_t src_addr = simple_strtoul(argv[2], NULL, 16);
    uint32_t set_offset = simple_strtoul(argv[3], NULL, 16);
    uint32_t spl_offset = simple_strtoul(argv[4], NULL, 16);
    uint32_t std_offset = simple_strtoul(argv[5], NULL, 16);

    if (packed_image_set_verify_all(src_addr) != 0) {
      puts("Source image set verification failed\n");
      return CMD_RET_FAILURE;
    }

    if (packed_image_set_write_all(src_addr, set_offset,
                                   spl_offset, std_offset) != 0) {
      puts("Failed to write image set\n");
      return CMD_RET_FAILURE;
    }

  } else if (strcmp(argv[1], "check") == 0) {

    if (argc != 4) {
      return CMD_RET_USAGE;
    }

    uint32_t set_offset = simple_strtoul(argv[2], NULL, 16);
    uint32_t ram_addr = simple_strtoul(argv[3], NULL, 16);

    if (image_set_verify_all(set_offset, ram_addr) != 0) {
      puts("Image set check failed\n");
      return CMD_RET_FAILURE;
    }

    puts("Image set check successful\n");

  } else if (strcmp(argv[1], "info") == 0) {

    if (argc != 4) {
      return CMD_RET_USAGE;
    }

    uint32_t set_offset = simple_strtoul(argv[2], NULL, 16);
    uint32_t ram_addr = simple_strtoul(argv[3], NULL, 16);

    if (image_set_info(set_offset, ram_addr) != 0) {
      puts("Image set info failed\n");
      return CMD_RET_FAILURE;
    }

  } else {
    /* Invalid subcommand */
    return CMD_RET_USAGE;
  }

  return CMD_RET_SUCCESS;
}
/***************************************************/

U_BOOT_CMD(
  image_set, CONFIG_SYS_MAXARGS, 0, do_image_set,
  "image set utility",
  "write <src> <set> <spl> <std>\n"
  "\tsrc: RAM address of packed image set to write\n"
  "\tset: target flash offset for image set header\n"
  "\tspl: target flash offset for SPL image\n"
  "\tstd: target flash offset for standard images\n"
  "check <set> <ram>\n"
  "\tset: flash offset of image set header to check\n"
  "\tram: RAM address used to load data\n"
  "info <set> <ram>\n"
  "\tset: flash offset of image set header to describe\n"
  "\tram: RAM address used to load data\n"
);
