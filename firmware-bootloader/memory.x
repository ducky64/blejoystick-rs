/* nRF52840 flash layout, shared with firmware-main/memory.x - keep in sync! */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH            : ORIGIN = 0x00000000, LENGTH = 32K   /* BOOTLOADER */
  BOOTLOADER_STATE : ORIGIN = 0x00008000, LENGTH = 4K
  ACTIVE           : ORIGIN = 0x00009000, LENGTH = 980K
  /* 0x000FE000 - 0x00100000: STORAGE (app bonding info), not touched by the bootloader */
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_active_start = ORIGIN(ACTIVE);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE);

__ram_start = ORIGIN(RAM);
__ram_end = ORIGIN(RAM) + LENGTH(RAM);
