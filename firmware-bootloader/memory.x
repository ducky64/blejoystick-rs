/* nRF52840 flash layout, shared with firmware-main/memory.x - keep in sync! */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH            : ORIGIN = 0x00000000, LENGTH = 32K   /* BOOTLOADER */
  BOOTLOADER_STATE : ORIGIN = 0x00008000, LENGTH = 4K
  ACTIVE           : ORIGIN = 0x00009000, LENGTH = 480K
  DFU              : ORIGIN = 0x00081000, LENGTH = 484K  /* ACTIVE + 1 page, required for swap */
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_active_start = ORIGIN(ACTIVE);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE);

__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
