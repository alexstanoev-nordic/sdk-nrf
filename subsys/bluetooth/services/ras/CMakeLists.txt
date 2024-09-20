#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library_sources_ifdef(
  CONFIG_BT_RAS
  ras_rd_buffer.c
  ras_rrsp.c
  ras_rrsp_rascp.c)
