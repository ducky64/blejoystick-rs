/* nRF52840 flash layout, shared with firmware-bootloader/memory.x - keep in sync! */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  BOOTLOADER       : ORIGIN = 0x00000000, LENGTH = 32K
  BOOTLOADER_STATE : ORIGIN = 0x00008000, LENGTH = 4K
  FLASH            : ORIGIN = 0x00009000, LENGTH = 480K  /* ACTIVE partition */
  DFU              : ORIGIN = 0x00081000, LENGTH = 484K  /* ACTIVE + 1 page, required for swap */
  /* 0x000FA000 - 0x000FE000 reserved */
  STORAGE          : ORIGIN = 0x000FE000, LENGTH = 8K    /* sequential-storage (bonding info) */
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

/* offsets relative to start of flash, used by embassy-boot */
__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOTLOADER);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOTLOADER);

__storage_start = ORIGIN(STORAGE) - ORIGIN(BOOTLOADER);
__storage_end = ORIGIN(STORAGE) + LENGTH(STORAGE) - ORIGIN(BOOTLOADER);
