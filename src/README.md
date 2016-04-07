## Compiling the sniffer yourself
### Installing dependencies
You will need to install the toolchain for compiling C/C++ code for the ARM Cortex platform and the corresponding libstdc++ library. On Ubuntu 15.04 and higher (only tested on Ubuntu 15.10), you should be able to just run the following:
``` bash
sudo apt-get install gcc-arm-none-eabi
```

On older Ubuntu versions, the required packages were not available in the standard Ubuntu repositories and you would have to install the compiler and the correspinding libstdc++ library yourself from somewhere else.

Other linux distros are of course also supported (tested on Arch Linux), but you will have to check for yourself which packages you need.

### Compiling and flashing sniffer
Make sure the [OpenBase](http://www.openmote.com/hardware/openbase.html) is connected.

Run the following commands (from the sniffer directory) to compile the code and then program it to the connected OpenMote:
``` bash
make
make bsl
```
