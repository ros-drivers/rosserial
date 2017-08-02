#ifndef ROS_MKL82Z_HARDWARE_H
#define ROS_MKL82Z_HARDWARE_H

#ifdef __cplusplus
extern "C" {
#include "mkl82z_uart.h"
}
class Mkl82zHardware {
public:
  Mkl82zHardware(){}

  void init(){
    MKL82Z_PIT_init();
    MKL82Z_UART_init();
  }

  int read(){
    return MKL82Z_UART_recbyte();
  }

  void write(uint8_t* data_buff, int length){
    uint16_t ii;
    for(ii=0;ii<length;ii++)
    {
      MKL82Z_UART_write_byte(data_buff[ii]);
    }
  }

  unsigned long time(){
    return MKL82Z_time();
  }
};

#endif /* __cplusplus */
#endif /* ROS_MKL82Z_HARDWARE_H */
