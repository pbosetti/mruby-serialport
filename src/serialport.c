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

#define IV_GET(name) mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, (name)))
#define IV_SET(name, value) mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, (name)), value)

enum error_src {
  TCGETADDR_ERROR = 1,
  TCSETADDR_ERROR
};


enum error_src
set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    return TCGETADDR_ERROR;

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;   // ignore break signal
  tty.c_lflag = 0;    // no signaling chars, no echo,
          // no canonical processing
  tty.c_oflag = 0;    // no remapping, no delays
  tty.c_cc[VMIN]  = 0;      // read doesn't block
  tty.c_cc[VTIME] = 5;      // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
          // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    return TCSETADDR_ERROR;
  else
    return 0;
}

enum error_src
set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    return TCGETADDR_ERROR;

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;      // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    return TCSETADDR_ERROR;
  else
    return 0;
}

mrb_value
mrb_serialport_open(mrb_state *mrb, mrb_value self)
{
  int fd;
  mrb_value mrb_portname = IV_GET("@port_name");
  mrb_value mrb_baud     = IV_GET("@baud");
  mrb_value mrb_blocking = IV_GET("@blocking");
  
  char *portname         = mrb_string_value_cstr(mrb, &mrb_portname);
  unsigned int baud      = mrb_fixnum(mrb_baud);
  
  fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    IV_SET("@error", mrb_fixnum_value(errno));
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not open port");
  }
  if (!isatty(fd)) {
    IV_SET("@error", mrb_fixnum_value(errno));
    mrb_raise(mrb, E_RUNTIME_ERROR, "Selected addres is not a tty");
  }
  if (set_interface_attribs(fd, baud, 0)) {
    IV_SET("@error", mrb_str_new_cstr(mrb, "Could not set attibutes"));
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not set attibutes");
  }
  if (set_blocking(fd, mrb_bool(mrb_blocking) ? 1 : 0) != 0) {
    IV_SET("@error", mrb_str_new_cstr(mrb, "Could not set blocking behavior"));
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not set blocking behavior");
  }
  IV_SET("@fd", mrb_fixnum_value(fd));
  return self;
}

mrb_value
mrb_serialport_close(mrb_state *mrb, mrb_value self)
{
  int fd = mrb_fixnum(IV_GET("@fd"));
  if (fd >= 0) {
    close(fd);
    IV_SET("@fd", mrb_fixnum_value(-1));
    return mrb_true_value();
  }
  else {
    return mrb_false_value();
  }
}

mrb_value
mrb_serialport_p_write(mrb_state *mrb, mrb_value self)
{
  mrb_value r_string;
  char *string;
  size_t l = 0;
  int fd = mrb_fixnum(IV_GET("@fd"));
  
  if (fd >= 0) {
    mrb_get_args(mrb, "S", &r_string);
    string = mrb_str_to_cstr(mrb, r_string);
    l      = write(fd, string, strlen(string));
    return mrb_fixnum_value(l);
  }
  else {
    return mrb_nil_value();
  }
}

mrb_value
mrb_serialport_p_read(mrb_state *mrb, mrb_value self)
{
  mrb_value r_buffer_size, r_result;
  size_t buf_len;
  char *string = NULL;
  int fd = mrb_fixnum(IV_GET("@fd"));
  
  if (fd >= 0) {
    mrb_get_args(mrb, "i", &r_buffer_size);
    buf_len = mrb_fixnum(r_buffer_size);
    string  = (char *)realloc(string, buf_len * sizeof(char));
    if (!string) {
      free(string);
      string = NULL;
      mrb_raise(mrb, E_RUNTIME_ERROR, "Could not allocate memory");
    }
    bzero(string, buf_len);
    buf_len  = read(fd, string, buf_len * sizeof(char));
    r_result = mrb_str_new_cstr(mrb, string);
    free(string);
    return r_result;
  }
  else {
    return mrb_nil_value();
  }
}

mrb_value
mrb_serialport_read_char(mrb_state *mrb, mrb_value self)
{
  char ch[1];
  int fd = mrb_fixnum(IV_GET("@fd"));
  if (fd >= 0) {
    read(fd, ch, sizeof(char));
    return mrb_str_new_cstr(mrb, ch);
  }
  else {
    return mrb_nil_value();
  }
}



void
mrb_mruby_serialport_gem_init(mrb_state* mrb)
{
  struct RClass *serialport_class;
  serialport_class = mrb_define_class(mrb, "SerialPort", mrb->object_class);
  mrb_define_method(mrb, serialport_class, "open", mrb_serialport_open, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "close", mrb_serialport_close, MRB_ARGS_NONE());
  mrb_define_method(mrb, serialport_class, "_write", mrb_serialport_p_write, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serialport_class, "_read", mrb_serialport_p_read, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serialport_class, "read_char", mrb_serialport_read_char, MRB_ARGS_NONE());
}

void
mrb_mruby_serialport_gem_final(mrb_state* mrb)
{
}
