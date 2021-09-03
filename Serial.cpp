#include "Serial.h"
 
int Serial_Send(const char *dev,unsigned char *buf,int len)
{
    int err;//返回调用函数的状态
    int length = -1;//记录发送的数据位数

    int fd = open(dev,O_RDWR | O_NOCTTY | O_NDELAY);
    if(-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (0);
    }

    //初始化串口
    do
    {
        err = UART_Init(fd,115200,0,8,1,'N');
    }while(FALSE == err || FALSE == fd);

    unsigned char buffer[Max] = {0x00};
    Format_Encoding(buffer,buf,len);//添加帧头帧尾

    for(int i = 0 ; i < 2 ; i++)
    {
        length = UART_Send(fd,buffer,len + 4);
        if( length == len + 4 )
        {
            std::cout << "Send Fail" << std::endl;
            break;
        }
    }
    
    UART_Close(fd);

    return length;
}

int Serial_Recv(const char *dev,unsigned char *res,int len)
{
    int err;//返回调用函数的状态
    int length = -1;//记录发送的数据位数

    int fd = open(dev,O_RDWR | O_NOCTTY | O_NDELAY);
    if(-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (0);
    }

    //初始化串口
    do
    {
        err = UART_Init(fd,115200,0,8,1,'N');
    }while(FALSE == err || FALSE == fd);

    unsigned char buffer[Max];

    length = UART_Recv(fd,res,len);//接收包括头帧和尾帧

    UART_Close(fd);

    return length;
}


void Format_Encoding(unsigned char *send_data,unsigned char *data,int len)
{
    unsigned char header[2] = { 0x55 , 0xaa };//设置头帧
    unsigned char ender[2] = { 0xcc , 0xbb };//设置尾帧

    //位处理
    for(int i = 0 ; i < 2 ; i++)//添加头帧
    {
        send_data[i] = header[i];
    }
    for(int i = 0 ; i < len ; i++)//存入数据
    {
        send_data[i + 2] = data[i];
    }
    for(int i = 0 ; i < 2 ; i++)//添加尾帧
    {
        send_data[i + len + 2] = ender[i];
    }

}

bool Data_Check(unsigned char *data,int len)
{
    if(data[0] != 0x55) return false;
    if(data[1] != 0xaa) return false;
    if(data[len - 2] != 0xcc) return false;
    if(data[len - 1] != 0xbb) return false;

    return true;
}


void UART_Close(int fd)
{
    close(fd);
}

int UART_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err=0;
    //设置串口数据帧格式
    if (UART_Set(fd,115200,0,8,1,'N') == FALSE)
    {
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

        case 0 ://不使用流控制
              options.c_cflag &= ~CRTSCTS;
              break;

        case 1 ://使用硬件流控制
              options.c_cflag |= CRTSCTS;
              break;
        case 2 ://使用软件流控制
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
                options.c_cflag |= CS5;
                break;
        case 6    :
                options.c_cflag |= CS6;
                break;
        case 7    :
                options.c_cflag |= CS7;
                break;
        case 8:
                options.c_cflag |= CS8;
                break;
        default:
                fprintf(stderr,"Unsupported data size\n");
                return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB;
                 options.c_iflag &= ~(INPCK | ICRNL | IXON);//该设置可以保证接收到所有数据
                 break;
        case 'o':
        case 'O'://设置为奇校验
                 options.c_cflag |= (PARODD | PARENB);
                 options.c_iflag |= INPCK;
                 break;
        case 'e':
        case 'E'://设置为偶校验
                 options.c_cflag |= PARENB;
                 options.c_cflag &= ~PARODD;
                 options.c_iflag |= INPCK;
                 break;
        case 's':
        case 'S': //设置为空格
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break;
        default:
                 fprintf(stderr,"Unsupported parity\n");
                 return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
                options.c_cflag &= ~CSTOPB; 
                break;
        case 2:
                options.c_cflag |= CSTOPB; 
                break;
        default:
                fprintf(stderr,"Unsupported stop bits\n");
                return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int UART_Send(int fd,unsigned char *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }

}

int UART_Recv(int fd, unsigned char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 1;
    time.tv_usec = 0;

    //串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    // printf("fs_sel = %d\n",fs_sel);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
        return len;
    }
    else
    {
        return FALSE;
    }
}
