class TimeoutError < RuntimeError; end

class SerialPort
  RATES = [2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400]
  
  attr_reader :port_name, :baud, :error, :errno
  attr_accessor :blocking, :buffer_size, :terminator
  attr_accessor :prompt, :autoflush, :timeout
  
  def initialize(port, baud=9600, blocking=false, &block)
    self.port_name = port
    self.baud      = baud
    @blocking      = blocking
    @error         = nil
    @errno         = nil
    @fd            = -1
    @buffer_size   = 1024
    @terminator    = "\r\n"
    @prompt        = ">"
    @autoflush     = false
    @timeout       = 10
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
  
  def read(buf_len = @buffer_size)
    self._read(buf_len)
  end

  def scan_upto(sep=@prompt)
    line = ""
    rng = (-sep.length)..(-1)
    start = Time.now
    loop do
      raise TimeoutError if Time.now - start > @timeout
      c = self.read_char
      if c then
        line << c[0]
        break if line[rng] == sep
      end
    end
    return line[0..-(sep.length + 1)]
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
  
  def command(cmd, prompt=@prompt, &block)
    case cmd
    when String
      self.flush if @autoflush
      self.write cmd
      start = Time.now
      while self.available == 0 do
        raise TimeoutError if Time.now - start > @timeout
      end
      result = self.scan_upto prompt
      if block_given? then
        yield result 
      else
        return result
      end
    when Array
      results = []
      cmd.each {|c| results << self.command(c, prompt, &block)}
      return results
    else
      raise ArgumentError, "Expect a String or an Array of Strings"
    end
  end
    
  def operate(&block)
    self.open if @fd < 0
    yield self
    self.close
  end
  
  def dtr_toggle
    self.dtr 1
    self.dtr 0
  end
  
end