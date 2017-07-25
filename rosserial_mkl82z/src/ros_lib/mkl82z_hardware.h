#ifndef ROS_MKL82Z_HARDWARE_H
#define ROS_MKL82Z_HARDWARE_H


class Mkl82zHardware {
  Mkl82zHardware();

  void init(){
  }

  int read(){
    return 0;
  }

  void write(uint8_t*, int length){
  }

  unsigned long time(){
    return 0;
  }
};

#endif /* ROS_MKL82Z_HARDWARE_H */
