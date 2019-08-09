#pragma once
#include <cstdint>


typedef struct{
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
  uint32_t period;
} DutyCycles;

class PRUComms
{
    public:
    PRUComms();
    ~PRUComms();
    void setDutyCycles(const DutyCycles& duty_cycles);

    private:
    uint8_t* _pru;
    int _fd;
};