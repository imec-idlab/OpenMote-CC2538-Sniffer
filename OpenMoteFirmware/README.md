This folder contains a modified version of the [firmware](https://github.com/OpenMote/firmware/) software from February 26, 2016, with the latest commit being [68a7e47](https://github.com/OpenMote/firmware/tree/68a7e470c3e8546f91431d2e08ededd736b7669a).

The following changes have been made:
* The practically empty README.md was replaced by this file
* tools/openmote-bsl/openmote-bsl.py was adapted to find the sniffer hex file
* tools/openmote-bsl/cc2538-bsl/cc2538-bsl.py was improved to support port=auto on Windows
* platform/cc2538/cc2538_linker.lds was changed to put the UDMA Channel Control Table at the front of the SRAM and to also make use of the 16K non-retention SRAM.
* platform/cc2538/libcc2538/libcc2538.py was rewritten to support Windows, no longer create new src files nor require git and to display the compile errors
* Files in platform/cc2538/libcc2538/src were renamed to fix problems on case-insensitive filesystems
* platform/cc2538/cc2538_include.h was changed to include the renamed header files from platform/cc2538/libcc2538/src
* Makefiles were changed to support Windows, run libcc2538.py when running make for the first time and copy the sniffer hex file
