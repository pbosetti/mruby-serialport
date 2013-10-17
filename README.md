SerialPort
==========

Experimental SerialPort communication for mruby.

This is a very early implementation. Only tested on Mac OS X.

Usage
-----

Typical usage:

    sp = SerialPort.new("/dev/ttys001", 115200).open
    sp.write "start"
    puts sp.read(100)
    sp.close

Block interface is also available:

    SerialPort.new("/dev/ttys001", 115200) do |sp|
      sp.write "start"
      puts sp.read(100)
    end # also closes the port
