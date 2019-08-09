#include "receiver.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

  Receiver::Receiver(const char * device)
  {
    _fd = open(device, O_RDWR | O_NONBLOCK | O_NOCTTY | O_SYNC);
    if (_fd < 0) {
      auto cached_err = strerror(errno);
      std::cerr << "Error opening " << device << ": " << cached_err;
      throw std::runtime_error("Invalid device");
    }
    /*baudrate 110, 8 bits, no parity, 1 stop bit */
    setInterfaceAttribs(B115200);
  }

  Receiver::~Receiver()
  {
    close(_fd);
  }


RxMessage Receiver::parseMessage(unsigned char * buf)
{
  RxMessage msg{};
  msg.header = buf[0];
  msg.length = buf[1];
  msg.type   = buf[2];
  parseSBUSArray(&buf[3], &buf[3] + 11, msg.data);
  parseSBUSArray(&buf[14], &buf[14] + 11, msg.data + 8);
  //get flag bs from bytes 25, 26
  msg.crc = buf[27];
  msg.footer = buf[28];
  return msg;
}

boost::optional<RxMessage> Receiver::getMessage()
{   
    unsigned char buf[80];
    int read_length = read(_fd, buf, sizeof(buf));
    if(read_length > 0)
    {
      return boost::optional<RxMessage>(parseMessage(buf));
    }
    return boost::optional<RxMessage>();
}


int Receiver::setInterfaceAttribs(int speed)
{
    struct termios tty;

    if (tcgetattr(_fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* enable parity bit */
    //tty.c_cflag &= ~PARODD;    /* even parity */
    tty.c_cflag &= ~CSTOPB;     /* need 2 stop bits */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}
