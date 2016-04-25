## Performance testing
The code in this folder was used to test the performance of the sniffer.

In the default mode, the program will send packets containing a 2 byte sequence number as fast as possible. Different patterns of packet sizes will be used (only small packets, only large packets, fully random packet sizes, ...). After a little over 250.000 packets (which takes about 7.5 minutes), all patterns will have been tested and the program will repeat itself.

You can easily test whether all packets were correctly received by taking your received packet count modulo 50.000. The number that you end up with should be the sequence number in the first two bytes of the last packet.

There are 2 defines at the top of the code that you can set.
- FIXED\_PACKET\_SIZE will change the program to only send packets of a single size the whole time
- MAX\_PERFORMANCE will change the program to get the maximum out of the hardware (packets will be identical, no more sequence number)

To recompile the program, run "make" and then run "make bsl" to flash it to the OpenMote.
