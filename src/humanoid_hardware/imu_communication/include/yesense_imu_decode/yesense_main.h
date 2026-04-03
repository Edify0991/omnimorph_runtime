#include <errno.h> /*错误号定义*/
#include <fcntl.h> /*文件控制定义*/
#include <getopt.h>
#include <stdio.h>  /*标准输入输出的定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h> /**/
#include <termios.h>   /*PPSIX 终端控制定义*/
#include <unistd.h>    /*UNIX 标准函数定义*/
#include <sys/ioctl.h> // 添加 ioctl 头文件
#include <iostream>
#include <string>
#include "analysis_data.h"

/*----------------------------------------------------------------------*/
#define TRUE 1
#define FALSE -1
#define RX_BUF_LEN 512

#define RESETSTR "\033[0m"
#define REDSTR "\033[31m"   /* Red */
#define GREENSTR "\033[32m" /* Green */

/*----------------------------------------------------------------------*/

class YesenseImu {
 public:
  YesenseImu(/* args */) {
    for (int i = 0; i < 512; i++) g_recv_buf[i] = 0;
    g_recv_buf_idx = 0;
    pos = 0;
    cnt = 0;
    speed_t speed = B921600;
    std::string imu_tty_usb_str = "/dev/ttyACM0";
    char const* dev = imu_tty_usb_str.data();
    fd = open(dev, O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
      std::cerr << REDSTR << "!!!!!!!!!! Can't Launch yesense IMU! " << RESETSTR << std::endl;
      exit(0);
    }

    // [大招] 模拟 DTR/RTS 复位，相当于软件层面的“拔插”
    // 这能有效解决设备状态残留问题
    int status;
    if (ioctl(fd, TIOCMGET, &status) != -1) {
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(fd, TIOCMSET, &status);
        usleep(100000); // 保持复位状态 100ms
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        ioctl(fd, TIOCMSET, &status);
    }
    usleep(200000); // 等待设备重启稳定

    std::cerr << GREENSTR << "Launch " << REDSTR << "Yesense" << GREENSTR << " IMU successful!" << RESETSTR << std::endl;

    // save to oldtio
    tcgetattr(fd, &oldtio);
    bzero(&newtio, sizeof(newtio));
    
    // 使用标准函数设置波特率
    cfsetispeed(&newtio, speed);
    cfsetospeed(&newtio, speed);

    // 设置控制模式
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CRTSCTS; // 显式关闭硬件流控

    // 设置输入模式
    newtio.c_iflag = IGNPAR;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // 显式关闭软件流控

    // 设置输出模式和本地模式
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;

    // 纯非阻塞模式，VMIN=0
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0; 
    
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSAFLUSH, &newtio) != 0) {
        std::cerr << REDSTR << "Setup serial port failed!" << RESETSTR << std::endl;
    }
    
    usleep(50000);
    tcflush(fd, TCIFLUSH);

    // [新增] 暴力清空缓冲区，读到没数据为止
    char trash[256];
    int trash_len;
    int retry = 0;
    do {
        trash_len = read(fd, trash, sizeof(trash));
        retry++;
    } while (trash_len > 0 && retry < 10);

    memset(buffer, 0, sizeof(buffer));
  }
  ~YesenseImu() { close(fd); }
  bool updateImuData();
  protocol_info_t getImuData() { return g_output_info; }

 private:
  int fd;
  int nread;
  char buffer[RX_BUF_LEN];
  struct termios oldtio;
  struct termios newtio;
  unsigned short cnt;
  int pos;
  protocol_info_t g_output_info;
  unsigned short g_recv_buf_idx;
  unsigned char g_recv_buf[512];
};
