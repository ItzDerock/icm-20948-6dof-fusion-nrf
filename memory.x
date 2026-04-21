MEMORY
{
  /* Adafruit bootloader reserves the first 0x26000 bytes */
  FLASH : ORIGIN = 0x00026000, LENGTH = 0x100000 - 0x26000
  RAM   : ORIGIN = 0x20000000, LENGTH = 0x40000
}
