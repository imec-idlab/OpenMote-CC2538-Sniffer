# IEEE 802.15.4 Sniffer for OpenMote-CC2538

This project provides a sniffer for the IEEE 802.15.4 network which can be run on the [OpenMote-CC2538](http://www.openmote.com/hardware/openmote-cc2538-en.html) hardware.

The sniffer is split in two components:
* Peripheral: The OpenMote that will capture the network traffic
* Host: This is the python script that runs on the PC

Features:
* The sniffer is **lossless**, all packets are captured
* The captured packets can be monitored **real-time** in Wireshark (or can be written to a pcap file)
* The sniffer is **cross-platform**, it will work on Windows, Linux and Mac OS X
* You can either show the FCS or replace it with the RSSI, LQI and a FCS Valid bit
* CRC checks and retransmissions are used on the serial connection to avoid possible corruption between the peripheral and host

To flash the sniffer to the OpenMote, a script is provided which only works when the mote is connected through an [OpenBase](http://www.openmote.com/hardware/openbase.html). The sniffer itself should however work with anything as long as the OpenMote is recognised as a serial device.

## Installing dependencies
The host software is a python script, so if you are a windows user you must first install python if you haven't already. The code has been written to be compatible with both python 2 and 3, so which version you have doesn't really matter.

One dependency that you will have to install is [pySerial](https://pypi.python.org/pypi/pyserial).

Windows users must also install [pywin32](https://sourceforge.net/projects/pywin32/files/pywin32/).

## Program your peripheral
If the OpenMote is connected to the PC with a OpenBase then just run the flash-bsl script:
``` bash
python flash-bsl.py
```

If you get the message `ERROR: Can't connect to target. Ensure boot loader is started.` then you will have to enter the Bootloader Backdoor first. If the software that is already flashed on the OpenMote supports it (e.g. this sniffer or an OpenWSN program) then you should be able to do this by just pressing the USER button. If all leds turned on after doing this and it still gives this error then press RESET and try again. If the USER button was not configured to flash the OpenMote then you will have to press the RESET button while the ON/SLEEP pin on the OpenBase is connected to the GND pin.

## Running the sniffer
Now that the peripheral is running the required software, it is time to run a host program that communicates with it over the USB cable (which is acting as a serial port).

When not provided any parameters, the script will start interactively and will ask you to input the necessary information (e.g. the channel to capture on).
``` bash
python sniffer.py
```

By default the script will start wireshark to provide real-time output. If you instead want to capture to a file then you should at least provide the ouput parameter.
``` bash
python sniffer.py -o output.pcap
```

For more information about which other parameters you can pass to the script, just run it with -h or --help.
``` bash
python sniffer.py -h
```

To pause the sniffer to change the channel or quit, just press the return key.

## FCS vs RSSI/LQI
Although the OpenMote provides the FCS in the TI CC24XX format, the host will recalculate the FCS before passing it to wireshark (when the checksum is correct).

If you want to see the TI CC24XX FCS (which contains the RSSI and LQI) in wireshark then you should pass the --replace-fcs option to the sniffer.

Wireshark supports both these variations, however only one can be used at a time. So if wireshark is parsing the FCS in the wrong way then you might want to change this wireshark setting.
If you already have packets then you can right click on the FCS on one of them and directly check/uncheck "TI CC24XX FCS format" under the Protocol Preferences menu. If you don't have any packets yet you can go to Edit > Preferences in the menu bar. In the list under Protocols, search for IEEE 802.15.4. Check/Uncheck the "TI CC24XX FCS format" option which you find here.

## Compiling the sniffer yourself
For instructions on how to recompile the sniffer, check the README.md file inside the src folder.

## Potential issues

### Linux: Permission denied: '/dev/ttyUSBX'
In order to use the sniffer you need to be able to write data to the serial USB port.

One way to do this is to run the script with root access (using 'sudo') but a more permanent and better solution is to give yourself permission to access the port. Add your user to the group that grants you this permission ('dialout' on Ubuntu, 'uucp' on Arch Linux) and log out afterwards to allow the changes to take effect.
``` bash
sudo gpasswd -a $USER dialout
```

Note that this error can also appear if you try to run the script directly after the OpenMote was plugged in. Wait a second and try again.

### Linux: Couldn't run /usr/bin/dumpcap in child process: Permission denied
Wireshark will not be able to capture data if you do not have the necessary permissions. Add your user to the 'wireshark' group and log out afterwards to allow the changes to take effect.
``` bash
sudo gpasswd -a $USER wireshark
```

### Windows: serial latency
The default serial settings on Windows cause relatively large delays in the communication. This may occasionally cause the buffer to get full and some packets to be dropped under very high network load. In order to reduce the latency, a small change is required in the settings of the COM port.

1. Control Panel -> Device Manager -> Ports (COM & LPT) -> {Select your USB Serial Port}
2. Right click and select "Properties"
3. Select the "Port Settings" tab
4. Click the "Advanced" button at the bottom
5. Change the Latency Timer (msec) to 1
6. Keep the USB Transfer Sizes at 4096
7. Save (and reboot)
