## Compiling the sniffer yourself
### Installing dependencies
TODO: Mention the required steps to be able to compile the firmware

### Preparation
If the OpenMoteFirmware folder is still empty then you should recursively update that submodule.
``` bash
git submodule update --init --recursive
```

A few small changes are required in the OpenMote firmware to make the compilation work, you can apply them by executing the following:
``` bash
patch -p0 < firmware-patch.diff
```

Finally generate the libcc2538.a file:
``` bash
cd OpenMoteFirmware/platform/cc2538/libcc2538
python libcc2538.py
ls .. | grep libcc2538.a
```

The third command should have printed "libcc2538.a" which means that the file was successfully generated. If it doesn't print anything then something went wrong in the python script. Open libcc2538.py with a text editor and remove the "stderr=devnull" parameter that you find somewhere in it. Now run the python script again and fix the errors that you receive.

### Compiling and flashing sniffer
Make sure the [OpenBase](http://www.openmote.com/hardware/openbase.html) is connected.

Run the following commands (from the sniffer directory) to compile the code and then program it to the connected OpenMote:
``` bash
make
sudo make bsl
```
