class SerialPort
  ERRNO_MAP = {
     1 => :EPERM,
     2 => :ENOENT,
     3 => :ESRCH,
     4 => :EINTR,
     5 => :EIO,
     6 => :ENXIO,
     7 => :E2BIG,
     8 => :ENOEXEC,
     9 => :EBADF,
    10 => :ECHILD,
    11 => :EDEADLK,
    12 => :ENOMEM,
    13 => :EACCES,
    14 => :EFAULT,
    15 => :ENOTBLK,
    16 => :EBUSY,
    17 => :EEXIST,
    18 => :EXDEV,
    19 => :ENODEV,
    20 => :ENOTDIR,
    21 => :EISDIR,
    22 => :EINVAL,
    23 => :ENFILE,
    24 => :EMFILE,
    25 => :ENOTTY,
    26 => :ETXTBSY,
    27 => :EFBIG,
    28 => :ENOSPC,
    29 => :ESPIPE,
    30 => :EROFS,
    31 => :EMLINK,
    32 => :EPIPE
  }
  
  RATES = [2400, 4800, 9600, 19200, 38400, 57600, 115200]
  
  attr_reader :port_name, :baud
  attr_accessor :blocking, :buffer_size
  
  def initialize(port, baud, blocking=true, &block)
    self.port_name = port
    self.baud      = baud
    @blocking      = blocking
    @error         = nil
    @fd            = -1
    @buffer_size   = 1024
    if block_given? then
      self.operate &block
    end
  end
  
  def baud=(v)
    v = v.to_i
    raise ArgumentError, "Wrong baud rate value #{v}" unless RATES.include? v
    @baud = v
  end
  
  def port_name=(v)
    @port_name = v
  end
  
  def error
    case @error
    when Fixnum
      ERRNO_MAP[@error]
    when String
      @error
    end
  end
  
  def puts(string)
    self.write(string.to_s + "\n")
  end
  
  def read(buf_len = @buffer_size)
    self._read(buf_len)
  end
  
  def operate(&block)
    self.open if @fd < 0
    yield self
    self.close
  end
  
end