# nautilus-datalogger-basic

The basic test setup is:

1. Format the flash chip, to make sure we don't have any weird stuff on it. 

Use 'fts_format' for this.  After uploading, it will go dark for a few seconds -- and then when it has erased everything successfully, it will 'double flash' every second (blink blink pause, blink blink pause).

2. Test out the basic datalogger functionality.

Use 'basic_datalogging' for this.  It will create a datafile (currently named 'data1.csv', can rename it as you like), and will simply log an integer to it every second (incrementing the integer every time).

3.  Test out reading the file back in via the serial monitor.

Use 'fatfs_print_file' for this.  It will wait until you open the serial port, and then it will print out all of the values of 'data1.csv' into the serial monitor.  You'll then need to copy-paste these values into another file.  (We can make this process more slick in the future, but for now I think it might actually be the most robust. If we end up having too many values in the serial monitor to do this easily, we can make workarounds -- but at least the data will be safely on the flash chip without being corrupted). 
