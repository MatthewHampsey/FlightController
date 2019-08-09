#include "pru_comms.h"
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <iostream>

PRUComms::PRUComms()
{
  _fd = open("/dev/mem", O_RDWR | O_SYNC);
  if(_fd < 0){
    std::cerr << "Can't open dev mem\n" << '\n';
    throw std::runtime_error("Can't open /dev/mem");
  }

  _pru = (uint8_t*)mmap(0, sizeof(DutyCycles), PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0x4A300000);
}

PRUComms::~PRUComms()
{
  close(_fd);
  munmap(_pru, 20);
}

void PRUComms::setDutyCycles(const DutyCycles& duty_cycles)
{
    memcpy(_pru, &duty_cycles, sizeof(DutyCycles));
}