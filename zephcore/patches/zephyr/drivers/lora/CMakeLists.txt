# SPDX-License-Identifier: Apache-2.0
# ZephCore patch: adds lr11xx driver subdirectory.

zephyr_sources_ifdef(CONFIG_LORA_SHELL shell.c)

add_subdirectory_ifdef(CONFIG_LORA_LR11XX lr11xx)

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_LORA_MODULE_BACKEND_LORAMAC_NODE loramac-node)
add_subdirectory_ifdef(CONFIG_LORA_MODULE_BACKEND_LORA_BASICS_MODEM lora-basics-modem)
add_subdirectory_ifdef(CONFIG_LORA_MODULE_BACKEND_NATIVE native)
# zephyr-keep-sorted-stop
