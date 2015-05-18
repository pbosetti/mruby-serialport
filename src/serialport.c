/*
** time.c - Time class
**
** See Copyright Notice in mruby.h
*/

#include "mruby.h"
#include "mruby/class.h"
#include "mruby/string.h"
#include "mruby/variable.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define IV_GET(name) mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, (name)))
#define IV_SET(name, value)                                                    \
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, (name)), value)

enum error_src { TCGETADDR_ERROR = 1, TCSETADDR_ERROR };

enum error_src set_interface_attribs(int fd, int speed, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
    return TCGETADDR_ERROR;

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag = IGNBRK | IGNCR;   // ignore break signal
  tty.c_lflag = 0;        // no signaling chars, no echo,
                          // no canonical processing
  tty.c_oflag = 0;        // no remapping, no delays
  tty.c_cc[VMIN] = 0;     // read doesn't block
  tty.c_cc[VTIME] = 5;    // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                     // enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    return TCSETADDR_ERROR;
  else
    return 0;
}

enum error_src set_blocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
    return TCGETADDR_ERROR;

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    return TCSETADDR_ERROR;
  fcntl(fd, F_SETFL, FNDELAY);
  return 0;
}

static void update_error(mrb_state *mrb, mrb_value self) {
  IV_SET("@error", mrb_str_new_cstr(mrb, strerror(errno)));
  IV_SET("@errno", mrb_fixnum_value(errno));
}

mrb_value mrb_serialport_open(mrb_state *mrb, mrb_value self) {
  int fd;
  mrb_value mrb_portname = IV_GET("@port_name");
  mrb_value mrb_baud = IV_GET("@baud");
  mrb_value mrb_blocking = IV_GET("@blocking");

  const char *portname = mrb_string_value_cstr(mrb, &mrb_portname);
  unsigned int baud = mrb_fixnum(mrb_baud);

  fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  if (!isatty(fd)) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  if (set_interface_attribs(fd, baud, 0)) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  if (set_blocking(fd, mrb_bool(mrb_blocking) ? 1 : 0) != 0) {
    IV_SET("@error", mrb_str_new_cstr(mrb, "Could not set blocking behavior"));
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not set blocking behavior");
  }
  IV_SET("@fd", mrb_fixnum_value(fd));
  return self;
}

mrb_value mrb_serialport_close(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  if (close(fd) == 0) {
    IV_SET("@fd", mrb_fixnum_value(-1));
    return mrb_true_value();
  }
  update_error(mrb, self);
  return mrb_false_value();
}

mrb_value mrb_serialport_p_write(mrb_state *mrb, mrb_value self) {
  char *string;
  size_t l = 0;
  ssize_t res = 0;
  int fd = mrb_fixnum(IV_GET("@fd"));
  
  mrb_get_args(mrb, "s", &string, &l);
  res = write(fd, (char *)string, l);
  if (res < 0) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(l);    
}

mrb_value mrb_serialport_test_write(mrb_state *mrb, mrb_value self) {
  char *string;
  size_t l = 0;
  ssize_t res = 0;
  int fd = mrb_fixnum(IV_GET("@fd"));
  
  mrb_get_args(mrb, "s", &string, &l);
  res = write(fd, (char *)string, l);
  printf("wrote '%s'", string);
  if (res < 0) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(l);    
}


mrb_value mrb_serialport_p_read(mrb_state *mrb, mrb_value self) {
  mrb_value r_buffer_size, r_result;
  ssize_t buf_len;
  char *string = NULL;
  int fd = mrb_fixnum(IV_GET("@fd"));

  mrb_get_args(mrb, "i", &r_buffer_size);
  buf_len = mrb_fixnum(r_buffer_size);
  string = (char *)realloc(string, buf_len * sizeof(char));
  if (!string) {
    free(string);
    string = NULL;
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not allocate memory");
  }
  bzero(string, buf_len);
  buf_len = read(fd, string, buf_len * sizeof(char));
  if ( buf_len == 0 ) {
    return mrb_nil_value();
  } 
  else if ( buf_len < 0 ) {
    if ( errno == EAGAIN )
      return mrb_nil_value();
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  } 
  r_result = mrb_str_new_cstr(mrb, string);
  free(string);
  return r_result;
}

mrb_value mrb_serialport_read_char(mrb_state *mrb, mrb_value self) {
  char ch[1];
  ssize_t len = 0;
  int fd = mrb_fixnum(IV_GET("@fd"));
  len = read(fd, ch, sizeof(char));
  if ( len == 0 ) {
      return mrb_nil_value();
  } 
  else if ( len < 0 ) {
    if ( errno == EAGAIN )
      return mrb_nil_value();
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  } 
  return mrb_str_new_cstr(mrb, ch);
}

mrb_value mrb_serialport_flush(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  if (tcflush(fd, TCIOFLUSH) != 0) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_true_value();
}

mrb_value mrb_serialport_available(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  int bytes_available;
  if (ioctl(fd, FIONREAD, &bytes_available) != 0) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  } 
  return mrb_fixnum_value(bytes_available);
}

mrb_value mrb_toggle_dtr(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  int val = 0, res = 0;
  int status = 0;
  mrb_get_args(mrb, "i", &val);
  if (val == 0) {
#ifdef TIOCCDTR 
    res = ioctl(fd, TIOCCDTR, NULL);
#elif defined(TIOCM_DTR)
    int iFlags;
    iFlags = TIOCM_DTR;
    res = ioctl(fd, TIOCMBIC, &iFlags);
#else
#error Dunno how to manage DTR
#endif
  }
  else if (val == 1) {
#ifdef TIOCCDTR 
    res = ioctl(fd, TIOCSDTR, NULL);
#elif defined(TIOCM_DTR)
    int iFlags;
    iFlags = TIOCM_DTR;
    res = ioctl(fd, TIOCMBIS, &iFlags);
#else
#error Dunno how to manage DTR
#endif
  }
  else {
    ioctl(fd, TIOCMGET, &status);
    return mrb_fixnum_value(status);  
  }
  if (res != 0) {
    update_error(mrb, self);
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_true_value();
}

void mrb_mruby_serialport_gem_init(mrb_state *mrb) {
  struct RClass *serialport_class;
  serialport_class = mrb_define_class(mrb, "SerialPort", mrb->object_class);
  mrb_define_method(mrb, serialport_class, "open", mrb_serialport_open, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "close", mrb_serialport_close, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "_write", mrb_serialport_p_write, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serialport_class, "_read", mrb_serialport_p_read, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serialport_class, "read_char", mrb_serialport_read_char, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "flush", mrb_serialport_flush, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "available", mrb_serialport_available, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "test_write", mrb_serialport_test_write, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serialport_class, "dtr", mrb_toggle_dtr, MRB_ARGS_REQ(1));
}

void mrb_mruby_serialport_gem_final(mrb_state *mrb) {}
