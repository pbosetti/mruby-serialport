SerialPort [![Build Status](https://travis-ci.org/pbosetti/mruby-serialport.svg?branch=master)](https://travis-ci.org/pbosetti/mruby-serialport)
==========

Experimental SerialPort communication for mruby.

This is a very early implementation. Only tested on Mac OS X.

Usage
-----

Typical usage:

```ruby
sp = SerialPort.new("/dev/ttys001", 115200).open
sp.write "start"
puts sp.read(100)
sp.close
```

Block interface is also available:

```ruby
SerialPort.new("/dev/ttys001", 115200) do |sp|
  sp.write "start"
  puts sp.read(100)
end # also closes the port
```