# Streaming EDL Terminal
Python terminal for Qualcomm EDL Streaming mode. Supports HDLC framing, eMMC read/write/erase, and Compatible with Windows 7, Linux

## OS
- Tested: Windows 7 x64, Windows 10 x64, GNU/Linux Ubuntu 22.04
- Not tested: MacOS

## Bugs v1.0
- 0x23 getstorageinfo command not working on lg g, nexus 4 maybe fixed on next release
- getstorageinfo supports not all MPRG programmers
- read supports not all MPRG programmers
## Bugs v1.1
- byte adressing not works, in v1.2 it will be fixed.

## Tested Devices

- LG Optimus L3
- LG Optimus L3 Dual
- LG Optimus L5
- LG Optimus L5 Dual
- LG Optimus L7
- LG Optimus L1 II
- LG Optimus L1 II Dual
- LG Optimus L3 II
- LG Optimus L3 II Dual
- LG Optimus L7 II
- LG Optimus L7 II Dual
- LG Nexus 4
- LG G
- Samsung G7102
- Samsung I9506
- Sony Xperia M

- Samsung Galaxy Ace Plus
- Samsung Galaxy Mini 2
- Samsung Galaxy S Duos
- Samsung Galaxy Young Duos

- HTC Desire C
- HTC Desire V
- HTC Desire SV
- HTC Desire X
- HTC Desire 200
- HTC Desire 300
- HTC Desire 500
- HTC Desire 600

## Installation & Setup on Windows

Follow these steps to get the project running:

1. **Install Python 3.8.10+**
   - Download and install Python 3.8.10+ from the [official website](https://www.python.org/downloads/release/python-3810/).
   - **Important:** Make sure to check the box **"Add Python to PATH"** during installation.

2. **Restart your PC**
   - You must restart your computer after installation to apply the system changes.

3. **Install Dependencies**
   - Open your terminal or Command Prompt and run:
   ```bash
   pip install pyserial

## Commands - v1.0

 -  exit                                          - Quit
 -  help                                          - Show this help
 -  load -f <file>                                - Load a HEX programmer file
 -  raw <bytes in HEX>                            - Send RAW hex bytes to device
 -  nop                                           - Ask what device is doing
 -  reboot                                        - Reboot phone
 -  initstorage                                   - Open eMMC card
 -  getstorageinfo                                - Get eMMC information
 -  read  -s <start_sector> -l <length> -f <file> - Read from RAM/eMMC
 -  write -s <start_sector> -f <file>             - Write data to eMMC
 -  erase -s <start_sector> -l <length>           - Write 00 to eMMC
 -  eraseall                                      - Erase all eMMC (CMD38)

## Commands - v1.1

  exit                                                                        - Quit
  help                                                                        - Show this help
  load -f <file> -n (newline)                                                 - Load a HEX/MBN/ELF programmer file
  raw <bytes in HEX>                                                          - Send RAW hex bytes to device
  nop                                                                         - Ask what device is doing (NOP)
  hello                                                                       - Get hello response
  bypass                                                                      - Bypass security
  close                                                                       - Close after init storage
  reboot                                                                      - Reboot phone
  initstorage                                                                 - Open eMMC card
  getstorageinfo                                                              - Get eMMC information
  readpartitiontable                                                          - Read partition table (short)
  readfullpartitiontable                                                      - Read partition table (all fields)
  read  -s <start_sector> -l <length> | -p <partition> -f <file> -n (newline) - Read from eMMC
  readhex -s <start_sector> -l <length> | -p <partition> [-l <length>]        - Read from eMMC and print as HEX
  write -s <start_sector> | -p <partition> -f <file> -n (newline)             - Write data to eMMC
  erase -s <start_sector> -l <length> | -p <partition> -n (newline)           - Write 00 to eMMC
  eraseall                                                                    - Erase all eMMC (CMD38)
  storagetest -s <start_sector> -l <length> | -p <partition> -n (newline)     - Write random data to eMMC and verify it
  safestoragetest -s <start_sector> -l <length> | -p <partition> -n (newline) - Same, then restore the original data
