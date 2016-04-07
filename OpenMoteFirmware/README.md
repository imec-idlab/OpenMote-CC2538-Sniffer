This folder contains a modified version of the firmware software from February 26, 2016, with the latest commit being [68a7e47](https://github.com/OpenMote/firmware/tree/68a7e470c3e8546f91431d2e08ededd736b7669a).

The following changes have been made:
* README.md was replaced by this file
* tools/openmote-bsl/openmote-bsl.py was adapted to find the sniffer hex file
* tools/openmote-bsl/cc2538-bsl/cc2538-bsl.py was improved to support port=auto on windows
* platform/cc2538/cc2538_linker.lds was changed to put the UDMA Channel Control Table at the front of the SRAM
* platform/cc2538/libcc2538/libcc2538.py was changed to not redirect output to /dev/null
* Makefile.include and platform/cc2538/Makefile.include were changed to run libcc2538.py automatically
* Makefile.include was changed to copy sniffer hex file to root directory
