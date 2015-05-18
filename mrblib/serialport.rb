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
  
  RATES = [2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400]
  
  attr_reader :port_name, :baud
  attr_accessor :blocking, :buffer_size, :terminator, :prompt
  
  def initialize(port, baud=9600, blocking=false, &block)
    self.port_name = port
    self.baud      = baud
    @blocking      = blocking
    @error         = nil
    @fd            = -1
    @buffer_size   = 1024
    @terminator    = "\r\n"
    @prompt = ">"
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
  
  def read(buf_len = @buffer_size)
    self._read(buf_len)
  end

  def scan_upto(sep=@prompt)
    line = ""
    rng = (-sep.length)..(-1)
    loop do
      c = self.read_char
      if c then
        line << c[0]
        break if line[rng] == sep
      end
    end
    return line
  end
  
  def read_lines(buf_len = @buffer_size)
    (self.read(buf_len) || '').split(@terminator)
  end
  
  def write(str)
    self._write(str.to_s)
  end
  
  def puts(string)
    self.write(string.to_s + @terminator)
  end
  
  def command(cmd, prompt=@prompt)
    raise ArgumentError, "Need a block" unless block_given?
    self.write cmd
    while self.available == 0 do 
      # no op
    end
    yield self.scan_upto prompt
  end
    
  def operate(&block)
    self.open if @fd < 0
    yield self
    self.close
  end
  
end