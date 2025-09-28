MAC Driver 

<a href="https://sparks.gogo.co.nz/assets/_site_/downloads/CH341SER_MAC_V1_8.zip">https://sparks.gogo.co.nz/assets/_site_/downloads/CH341SER_MAC_V1_8.zip</a>

wchisp

<a href="https://github.com/ch32-rs/wchisp/releases">https://github.com/ch32-rs/wchisp/releases</a>

extract to folder

mod command

```bash
chmod +x wchisp
```

show usb command

use /dev/tty.wchusbserialXXXXX


```bash
 ls /dev/tty.*
```

Unlock Command

```bash
./wchisp -s -p /dev/tty.wchusbserial14710 -b Baud1m config unprotect
```

output example 

```
07:56:01 [INFO] Opening serial port: "/dev/tty.wchusbserial14710" @ 115200 baud
07:56:01 [INFO] Switching baudrate to: 1000000 baud
07:56:01 [INFO] Code Flash unprotected
07:56:01 [INFO] Device reset
```

Flash command

```bash
./wchisp -s -p /dev/tty.wchusbserial14710 -b Baud1m config unprotect
```

output example

```
07:54:26 [INFO] Opening serial port: "/dev/tty.wchusbserial14710" @ 115200 baud
07:54:26 [INFO] Switching baudrate to: 1000000 baud
07:54:26 [INFO] Chip: CH32V203C8T6[0x3119] (Code Flash: 64KiB)
07:54:26 [INFO] Chip UID: CD-AB-C6-C1-D0-BD-63-2B
07:54:26 [INFO] BTVER(bootloader ver): 02.70
07:54:26 [INFO] Code Flash protected: false
07:54:26 [INFO] Current config registers: a55a3fc000ff00ffffffffff00020700cdabc6c1d0bd632b
RDPR_USER: 0xC03F5AA5
  [7:0]   RDPR 0xA5 (0b10100101)
    `- Unprotected
  [16:16] IWDG_SW 0x1 (0b1)
    `- IWDG enabled by the software, and disabled by hardware
  [17:17] STOP_RST 0x1 (0b1)
    `- Disable
  [18:18] STANDBY_RST 0x1 (0b1)
    `- Disable, entering standby-mode without RST
  [23:22] SRAM_CODE_MODE 0x0 (0b0)
    `- CODE-192KB + RAM-128KB / CODE-128KB + RAM-64KB depending on the chip
DATA: 0xFF00FF00
  [7:0]   DATA0 0x0 (0b0)
  [23:16] DATA1 0x0 (0b0)
WRP: 0xFFFFFFFF
  `- Unprotected
07:54:26 [INFO] Read ./V0020-MOD/V0020-A-EXT-700MM.bin as Binary format
07:54:26 [INFO] Firmware size: 41984
07:54:26 [INFO] Erasing...
07:54:26 [INFO] Erased 42 code flash sectors
07:54:27 [INFO] Erase done
07:54:27 [INFO] Writing to code flash...
```