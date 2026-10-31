/* Reserve the final two sectors without adding them to the UF2 payload. */
ASSERT(__flash_binary_end <= ORIGIN(FLASH) + LENGTH(FLASH) - 8192,
       "Name Tag firmware overlaps persistent settings (last 8192 flash bytes)")
