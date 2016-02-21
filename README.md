# IEEE 802.15.4 Sniffer for OpenMote-CC2538

This project provides a lossless sniffer for the IEEE 802.15.4 network which can be run on the [OpenMote-CC2538](http://www.openmote.com/hardware/openmote-cc2538-en.html) hardware.

## Setup
Download the source code and recursively update the submodules:
``` bash
git clone https://bitbucket.org/texus/openmote-cc2538-802.15.4-sniffer
git submodule update --init --recursive
```

A few small changes are required in the OpenMote firmware to make the sniffer work, you can apply them by executing the following:
``` bash
patch -p0 < bsl-patch.diff
```

Finally generate the libcc2538.a file:
``` bash
cd OpenMoteFirmware/platform/cc2538/libcc2538
python libcc2538.py
ls .. | grep libcc2538.a
```

The third command should have printed "libcc2538.a" which means that the file was successfully generated. If it doesn't print anything then something went wrong in the python script. Open libcc2538.py with a text editor and remove the "stderr=devnull" parameter that you find somewhere in it. Now run the python script again and fix the errors that you receive.


## Compiling and programming sniffer
Make sure the [OpenBase](http://www.openmote.com/hardware/openbase.html) is connected.

Run the following commands (from the sniffer directory) to compile the code and then program it to the connected OpenMote:
``` bash
make
sudo make bsl
```

## Running
``` bash
sudo python serial-connect.py
```

## Windows latency issue
When running the sniffer on Windows, a small change is required in the settings of the COM port to achieve maximum performance. Without the change the sniffer may not be able to capture all packets under very high network load.
1. Control Panel -> Device Manager -> Ports (COM & LPT) -> {Select your USB Serial Port}
2. Right click and select "Properties"
3. Select the "Port Settings" tab
4. Click the "Advanced" button at the bottom
5. Change the Latency Timer (msec) to 1 (or the lowest possible setting)
6. Change the USB Transfer Sizes to 64 (or the lowest possible settings)
7. Save and close

## Current modules
- sniffer  
    The actual sniffer program
- custom-TX  
    Transmits packets with various lengths as fast as possible
