Android Serial GPS Driver

How to use:

To set serial port, add a property "ro.kernel.android.gps" and set it equal to your GPS device file.
ie. ro.kernel.android.gps=ttyO1

Default baud rate is 9600, to adjust add a property "ro.kernel.android.gpsttybaud" and set it equal to the needed rate. (4800-115200)
ie. ro.kernel.android.gpsttybaud=9600

Notes:
* If using a USB device make sure you have the necessary kernel modules loaded or built in to the kernel.
* Make sure the permissions on your device file are correct

Donate:
If you find any of this useful and want to show appreciation see below:

PayPal: keith.conger@gmail.com
Bitcoin: 1Pg54vVnaLxNsziA6cy9CTefoEG5iAm9Uh
