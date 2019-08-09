#pragma once
#include <termios.h>
#include <cstdint>
#include <assert.h>
#include <algorithm>
#include <boost/optional/optional.hpp>

struct RxMessage
{
  uint8_t header;
  uint8_t length;
  uint8_t type;
  int16_t data[16];
  uint8_t flags;
  uint8_t RSSI;
  uint8_t crc;
  uint8_t footer;
};

class Receiver
{
    public:
    Receiver(const char * device);
    ~Receiver();
    RxMessage parseMessage(unsigned char * buf);
    int setInterfaceAttribs(int speed);
    boost::optional<RxMessage> getMessage();


template <typename Iterator, typename OutputIterator>
void parseSBUSArray(Iterator begin, Iterator end, OutputIterator result)
{
  assert(std::distance(begin, end) > 7);
  *result = *begin;
  ++begin;
  *result |= ((*begin & 0b00000111) << 8);
  ++result;
  *result  = ((*begin & 0b11111000) >> 3);
  ++begin;
  *result |= ((*begin & 0b00111111) << 5); 
  ++result;
  *result  = ((*begin & 0b11000000) >> 6);
  ++begin;
  *result |= (*begin << 2);
  ++begin;
  *result |= ((*begin & 0b00000001) << 10); 
  ++result;
  *result  = ((*begin & 0b11111110) >> 1);
  ++begin;
  *result |= ((*begin & 0b00001111) << 7); 
  ++result;
  *result  = ((*begin & 0b11110000) >> 4);
  begin++;
  *result |= ((*begin & 0b01111111) << 4); 
  ++result;
  *result  = ((*begin & 0b10000000) >> 7);
  begin++;
  *result |= (*begin << 1);
  begin++;
  *result |= ((*begin & 0b00000011) << 9);
  ++result;
  *result  = ((*begin & 0b11111100) >> 2);
  begin++;
  *result |= ((*begin & 0b00011111) << 6);
  ++result;
  *result  = ((*begin & 0b11100000) >> 5);
  begin++;
  *result |= (*begin << 3);
}


    private:
    int _fd;
    
};

