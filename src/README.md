## Compiling the sniffer yourself
### Installing dependencies
You will need to install the toolchain for compiling C/C++ code for the ARM Cortex platform.

#### Linux
On Ubuntu 15.04 and higher (only tested on Ubuntu 15.10), you should be able to just run the following:
``` bash
sudo apt-get install gcc-arm-none-eabi
```

Other linux distros are of course also supported (tested on Arch Linux), but you will have to check for yourself which packages you need.

#### Mac OS X
One way to install the dependencies would be to use brew:
``` bash
brew tap PX4/homebrew-px4
brew update
brew install gcc-arm-none-eabi-49
```

#### Windows
Download and install both [GCC ARM](https://launchpad.net/gcc-arm-embedded/+download) and [Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm).


### Compiling and flashing sniffer
Make sure the [OpenBase](http://www.openmote.com/hardware/openbase.html) is connected.

Run the following commands to compile the code and then program it to the connected OpenMote:
``` bash
make
make bsl
```
