#include "myusart.h"

uint8_t tim_endflag;
uint8_t usart1rebuf[60],usart1relen,flagFrameusart1;
uint8_t rxbuf_uart1[60],txbuf_uart1[1300],sendbuf[1300],sendlen;

uint8_t flag_cmd;//指令标志

uint8_t packnum;//原始数据包号

#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
/**
 * @brief	定义_sys_exit()以避免使用半主机模式
 *
 * @param	void
 *
 * @return  void
 */
void _sys_exit(int x)
{
    x = x;
}
/**
 * @brief	重定义fputc函数
 *
 * @param	ch		输出字符量
 * @param	f		文件指针
 *
 * @return  void
 */
int fputc(int ch, FILE *f)
{
    while((USART1->ISR & 0X40) == 0); //循环发送,直到发送完毕

    USART1->TDR = (uint8_t) ch;
    return ch;
}
#endif

long FloatToHex(float HEX)
{
  return *(long *)&HEX;
}

float HexToFloat(long FLOAT)
{
  return *(float *)&FLOAT;
}
// 总加和校验
uint8_t checksum(uint8_t *buf, uint16_t len)
{
  uint16_t sum = 0, i = 0;
  for (i = 6; i < len; i++)
  {
    sum += buf[i];
  }
  sum = sum % 256;
  return sum;
}

void cmd32driver()//获取采样最终结果
{
	uint8_t cmd32buf[] = {0xA5,0xA5,0x50,0x32,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00};
	//									0		1		2		3		4		5		6		7		8		9		10	11
	//									|帧头		  |命令		   |len			|结果|S值	|L值	|S值	|L值	|校验
//	cmd32buf[6] = 0;//漏水结果
//	cmd32buf[7] = 0;//S计算值
//	cmd32buf[8] = 0;//L计算值
//	cmd32buf[9] = 0;//S阈值
//	cmd32buf[10] = 0;//L阈值
	cmd32buf[6] = 1;//漏水结果
	cmd32buf[7] = 3;//S计算值
	cmd32buf[8] = 80;//L计算值
	cmd32buf[9] = 10;//S阈值
	cmd32buf[10] = 0x20;//L阈值
	cmd32buf[11] = checksum(cmd32buf,11);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd32buf, 12, 0xffff);
}

void cmd33driver()//获取采样原始数据
{
	uint8_t cmd33buf[1261];
	uint16_t i;
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); 	//红灯亮
	cmd33buf[0] = 0xA5;//帧头
	cmd33buf[1] = 0xA5;//帧头
	cmd33buf[2] = 0x50;//命令
	cmd33buf[3] = 0x33;//命令
	cmd33buf[4] = 0x04;//数据长度1254
	cmd33buf[5] = 0xE6;//数据长度1254
	cmd33buf[6] = 0;//总包数160
	cmd33buf[7] = 0xA0;//总包数160
	cmd33buf[8] = 0;//当前包号
	cmd33buf[9] = packnum;//当前包号
	for(i=0;i<1250;i++)
	{
		cmd33buf[10+i] = Sram_Read_Compare(packnum,i);
	}
	cmd33buf[1260] = checksum(cmd33buf,1260);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd33buf, 1261, 0xffff);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); 	//红灯灭
}

void cmd35driver()//预关机命令
{
	uint8_t cmd35buf[] = {0xA5,0xA5,0x50,0x35,0x00,0x01,0x00,0x00};
	//									0		1		2		3		4		5		6		7		
	//									|帧头		  |命令		   |len			|结果|校验	
	cmd35buf[6] = 0;//预关机结果	正常：0x1,故障：0x0

	cmd35buf[7] = checksum(cmd35buf,7);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd35buf, 8, 0xffff);
}

void cmd36driver()//设备自检
{
	uint8_t cmd36buf[] = {0xA5,0xA5,0x50,0x36,0x00,0x04,0x00,0x00,0x00,0x00,0x00};
	//									0		1		2		3		4		5		6		7		8		9		10
	//									|帧头		  |命令		   |len			|结果|S值	|L值	|S值	|校验
	cmd36buf[6] = 0;//设备自检结果	成功：0x1,失败：0x0
	cmd36buf[7] = 0;//FLASH自检结果	成功：0x1,失败：0x0
	cmd36buf[8] = 0;//PSRAM自检结果	成功：0x1,失败：0x0
	cmd36buf[9] = 0;//PGA自检结果	成功：0x1,失败：0x0

	cmd36buf[10] = checksum(cmd36buf,10);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd36buf, 11, 0xffff);
}
void cmd40driver()//发20W数据
{
	uint8_t cmd40buf[1260];
	uint16_t i;
	packnum = 1;
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); 	//红灯亮
	for(packnum=0;packnum<160;packnum++)
	{
		for(i=0;i<1250;i++)
		{
//			cmd40buf[i] = Sram_Read_Compare(packnum,i);
			printf("%d,",Sram_Read_Compare(packnum,i));
		}
//		HAL_UART_Transmit(&huart1, (uint8_t *)cmd40buf, 1250, 0xffff);
	}
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); 	//红灯灭
}

//计算发送的数据长度，并且将数据放到*buf数组中                     
uint8_t UART1_Read(uint8_t *buf, uint8_t len)  
{
  uint8_t i;
  if(len>usart1relen)  //指定读取长度大于实际接收到的数据长度时
  {
    len=usart1relen; //读取长度设置为实际接收到的数据长度
  }
  for(i=0;i<len;i++)  //拷贝接收到的数据到接收指针中
  {
    *buf=usart1rebuf[i];  //将数据复制到buf中
    buf++;
  }
  usart1relen=0;              //接收计数器清零
  return len;                   //返回实际读取长度
}

//中断处理函数
void UART1_CallBack(void)
{
	if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET))
	{
		uint8_t Res;
		HAL_UART_Receive(&huart1, &Res, 1, 1000);
		usart1rebuf[usart1relen] = Res;
		usart1relen++;
		if(usart1relen > 59) usart1relen = 59;
		//HAL_UART_IRQHandler(&huart1);
	}
}

void Uart1RxMonitor(uint8_t ms) //接收监控
{
  static uint8_t USART1_RX_BKP=0;  //定义USART2_RC_BKP暂时存储诗句长度与实际长度比较
  static uint16_t idletmr=0;        //定义监控时间
  if(usart1relen>0)//接收计数器大于零时，监控总线空闲时间
  {
    if(USART1_RX_BKP!=usart1relen) //接收计数器改变，即刚接收到数据时，清零空闲计时
    {
      USART1_RX_BKP=usart1relen;  //赋值操作，将实际长度给USART2_RX_BKP
      idletmr=0;                    //将监控时间清零
    }
    else                              ////接收计数器未改变，即总线空闲时，累计空闲时间
    {
      //如果在一帧数据完成之前有超过3.5个字节时间的停顿，接收设备将刷新当前的消息并假定下一个字节是一个新的数据帧的开始
      if(idletmr<2)                  //空闲时间小于1ms时，持续累加
      {
        idletmr +=ms;
	if(idletmr>=2)             //空闲时间达到1ms时，即判定为1帧接收完毕
	{
          flagFrameusart1=1;//设置命令到达标志，帧接收完毕标志
          uart1driver();
	}
      }
    }
  }
  else
  {
    USART1_RX_BKP=0;
  }
}


uint8_t cmd32flag;
void uart1driver()
{
	static uint8_t len,sum;
	if(flagFrameusart1 == 1)
	{
		flagFrameusart1 = 2;
		len = UART1_Read(rxbuf_uart1,sizeof(rxbuf_uart1));   //将接收到的命令读到缓冲区中
		if(rxbuf_uart1[0]==0xA5 && rxbuf_uart1[1]==0xA5 && rxbuf_uart1[2]==0x5A)//2帧头+1下行位
		{
			sum = checksum(rxbuf_uart1,len-1);
			if(rxbuf_uart1[len-1] == sum)//加和校验
			{
				switch (rxbuf_uart1[3])  //按功能码执行操作
				{
					case 0x31://采样开始命令
					{
						flag_cmd = 0x31;
						calistatus = 2;//收到指令，正在采样
					} break;
					case 0x32://获取采样最终结果
					{
						flag_cmd = 0x32;
//						cmd32flag++;
						if(calistatus == 1)calistatus = 2;//收到指令，正在采样
//						if(calistatus == 2)calistatus = 3;//收到指令，采样完成
					} break;
					case 0x33://获取采样原始数据
					{
						flag_cmd = 0x33;
						packnum = rxbuf_uart1[6];//所需数据包号
					} break;
					case 0x35://预关机命令
					{
						flag_cmd = 0x35;
					} break;
					case 0x36://设备自检
					{
						flag_cmd = 0x36;
					} break;
					case 0x40://发20W数据
					{
						flag_cmd = 0x40;
					} break;
				}
			}
		}
	}
}

void uart1cmddriver()
{
	//假串口CTS硬流控
	static uint8_t IO1status;//低电平才发送数据
	IO1status = HAL_GPIO_ReadPin(IO1_GPIO_Port, IO1_Pin);
	switch(flag_cmd)
	{
		case 0x31://采样开始命令
		{
			flag_cmd = 0;
			 
		} break;
		case 0x32://获取采样最终结果
		{
			flag_cmd = 0;
//			if(cmd32flag>1)
			{
				
//				if(IO1status==0) cmd32driver();
				cmd32driver();
				if(calistatus == 2)
				{
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); 	//红灯亮
					HAL_TIM_Base_Start_IT(&htim2);// 启动定时器，开始采集
				}
			}
			
			
		} break;
		case 0x33://获取采样原始数据
		{
			flag_cmd = 0;
//			if(IO1status == 0) cmd33driver();
			cmd33driver();
		} break;
		case 0x35://预关机命令
		{
			flag_cmd = 0;
			if(IO1status == 0) cmd35driver();
		} break;
		case 0x36://设备自检
		{
			flag_cmd = 0;
			if(IO1status == 0) cmd36driver();
		} break;
		case 0x40://发20W数据
		{
			flag_cmd = 0;
			cmd40driver();
		} break;
	}
}

















