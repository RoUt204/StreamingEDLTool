# DmssEDLTerminal
Python terminal for Qualcomm EDL Streaming mode. Supports HDLC framing, eMMC read/write/erase, and Compatible with Windows 7.

## OS
- Tested: Windows 7 x64
- Not tested: Linux, MacOS

## Bugs
- 0x23 getstorageinfo command not working on lg g, nexus 4 maybe fixed on next release
- getstorageinfo supports not all MPRG programmers
- read supports not all MPRG programmers

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

1. **Install Python 3.8.10**
   - Download and install Python 3.8.10 from the [official website](https://www.python.org/downloads/release/python-3810/).
   - **Important:** Make sure to check the box **"Add Python to PATH"** during installation.

2. **Restart your PC**
   - You must restart your computer after installation to apply the system changes.

3. **Install Dependencies**
   - Open your terminal or Command Prompt and run:
   ```bash
   pip install pyserial

## Commands

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

