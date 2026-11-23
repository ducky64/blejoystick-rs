/* nRF52840 flash layout, shared with firmware-bootloader/memory.x - keep in sync! */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  BOOTLOADER       : ORIGIN = 0x00000000, LENGTH = 32K
  BOOTLOADER_STATE : ORIGIN = 0x00008000, LENGTH = 4K
  FLASH            : ORIGIN = 0x00009000, LENGTH = 980K  /* ACTIVE partition, overwritten in place by the bootloader's DFU */
  STORAGE          : ORIGIN = 0x000FE000, LENGTH = 8K    /* sequential-storage (bonding info) */
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

/* flash addresses (flash starts at 0, so these are also NVMC offsets), same convention as firmware-bootloader/memory.x */
__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__storage_start = ORIGIN(STORAGE);
__storage_end = ORIGIN(STORAGE) + LENGTH(STORAGE);
