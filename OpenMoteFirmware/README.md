This folder contains the firmware software as it was on February 26, 2016, with the latest commit being [68a7e47](https://github.com/OpenMote/firmware/tree/68a7e470c3e8546f91431d2e08ededd736b7669a).

The following changes have however been made:
- The README.md file was replaced by this file
- The tools/openmote-bsl/openmote-bsl.py file was adapted to find the sniffer hex file
- The platform/cc2538/cc2538_linker.lds was changed to put the UDMA Channel Control Table at the front of the SRAM
