#ifndef  _USART_H
#define  _USART_H
 
//串口相关的头文件    
#include <stdio.h>      /*标准输入输出定义*/    
#include <stdlib.h>     /*标准函数库定义*/    
#include <unistd.h>     /*Unix 标准函数定义*/    
#include <sys/types.h>     
#include <sys/stat.h>       
#include <fcntl.h>      /*文件控制定义*/    
#include <termios.h>    /*PPSIX 终端控制定义*/    
#include <errno.h>      /*错误号定义*/    
#include <string.h>         
#include <time.h>
#include <string.h>   
#include <iostream>
     
//宏定义
#define TRUE   0    
#define FALSE  -1  

#define Max  256   //数组的最大容量

struct ReceiveInformation
{
    //全场定位信息
    float X = 0.0f;//全场定位位于世界的x轴坐标
    float Y = 0.0f;//全场定位位于世界的y轴坐标
    float theta = 0.0f;//全场定位的旋转角度（角度制）
};

/*串口数据类型转换共用体*/
template <class T>
union Swap   //发送数据转换使用
{
    T s;
    unsigned char swap[sizeof(T) / sizeof(unsigned char)];
};


/*串口调用函数*/
/**
* @brief send the data to STM32
* @param dev : the name of the serial port
* @param buf : the data you need to send to STM32
* @param len : the length of the data
* @return length : the length of the sending data (Returns -1 if sending fails)
*/
int Serial_Send(const char *dev,unsigned char *buf,int len);


/**
* @brief Receive the data from STM32
* @param dev : the name of the serial port
* @param res : Saves an array of received data from STM32
* @param len : the length of the data
* @return length : the length of the receive data (Returns -1 if sending fails)
*/
int Serial_Recv(const char *dev,unsigned char *res,int len);


/**
* @brief Parse the data from STM32   ;   transform unsigned char to short
* @param array : the unsigned char is needed to parse
* @return return the short
*/
template <class T>
T Parse_Data(unsigned char *array ,int index)
{
    Swap<T> t;
    for(int i = 0 ; i < sizeof(T) / sizeof(unsigned char) ; i++)
    {
        t.swap[i] = array[index + i];
    }
    return t.s;
}


/*串口通讯子函数*/
/**
* @brief Add frameheads and ends of frames
* @param send_data : Store encoded data
* @param data : The data that needs to be sent
* @param len : the length of the data
*/
void Format_Encoding(unsigned char *send_data,unsigned char *data,int len);

/**
* @brief Check that the data is in the correct format
* @param data : The data that needs to be check
* @param len : the length of the data
* @return true : right   false : incorrect
*/
bool Data_Check(unsigned char *data,int len);


/**
* @brief Close the serial port
* @param fd : Serial file descriptor
*/
extern void UART_Close(int fd);


/**
* @brief Set the serial port
* @param fd : Serial file descriptor
* @param speed : Porter rate
* @param flow_ctrl : Data flow control
* @param databits : The data bits are valued at 7 or 8
* @param stopbits : Stop bits have a value of 1 or 2
* @param parity : The effect type is valued 'N', 'E', 'O', 'S'
* @return 1 : true
* @return 0 : false
*/
extern int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);


/**
* @brief Initialize the serial port
* @param fd : Serial file descriptor
* @param speed : Porter rate
* @param flow_ctrl : Data flow control
* @param databits : The data bits are valued at 7 or 8
* @param stopbits : Stop bits have a value of 1 or 2
* @param parity : The effect type is valued 'N', 'E', 'O', 'S'
* @return 1 : true
* @return 0 : false
*/
extern int UART_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);


/**
* @brief Receive serial data
* @param fd : Serial file descriptor
* @param rcv_buf : Receive data buffer
* @param data_len : the length of the data
* @return length : The length of the data received
* @return -1 : Receive false
*/
extern int UART_Recv(int fd, unsigned char *rcv_buf,int data_len);


/**
* @brief Send serial data
* @param fd : Serial file descriptor
* @param send_buf : Data sent serially is required
* @param data_len : the length of the data
* @return length : The length of the data
* @return -1 : Send false
*/
extern int UART_Send(int fd, unsigned char *send_buf,int data_len);


#endif



