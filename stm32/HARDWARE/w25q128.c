#include "w25q128.h"

extern SPI_HandleTypeDef hspi2;

W25Qxx_FlashInfoTypeDef W25QxxInfo;   //  W25Qxx 相关信息

uint8_t  W25QxxWriteBuffer[4096];   //  用于写W25Qxx芯片扇区数据缓存

void W25Q128_CS_LOW(void) {
    HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_RESET);
}

void W25Q128_CS_HIGH(void) {
    HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_SET);
}

uint8_t W25Qxx_Read_Byte(void)
{
	    uint8_t r_date  ;
	    HAL_SPI_Receive(&hspi2,&r_date,1,1);
	    return r_date ;   
}

void W25Qxx_Write_Byte(uint8_t t_date)
{
	    HAL_SPI_Transmit(&hspi2,&t_date,1,1);
}

/********************************************************************************
	*方法名称： uint8_t W25Qxx_Read_Buffer(uint8_t  *buffer , uint16_t len)
	*功能：     SPI读多个字节到缓存
  *参数：     buffer ：数据缓存 ， len :读取数据的长度
	*返回：			r_date:读到的数据
*********************************************************************************/
HAL_StatusTypeDef W25Qxx_Read_Buffer(uint8_t  *buffer , uint16_t len)
{  
	  return   HAL_SPI_Receive(&hspi2,buffer,len,100);  
	  //	   return  HAL_OK  ;
}


/********************************************************************************
	*方法名称： void W25Qxx_Write_Buffer(uint8_t date)
	*功能：     SPI写多个字节
	*参数：      buffer ：数据缓存 ， len :读取数据的长度
	*返回：			操作状态
*********************************************************************************/
HAL_StatusTypeDef W25Qxx_Write_Buffer(uint8_t  *buffer , uint16_t len)
{
	  return   HAL_SPI_Transmit(&hspi2,buffer,len,100);  
	  //	   return  HAL_OK  ;
}

/********************************************************************************
	*返回：			芯片型号： W25Qxx_16  =	0XEF14  
												 W25Qxx_32  =	0XEF15  
												 W25Qxx_64  =	0XEF16  
												 W25Qxx_128 =	0XEF17 	
*********************************************************************************/ 
uint16_t W25Qxx_Read_ID(void)
{
	uint16_t id = 0;
	W25Q128_CS_LOW();
	W25Qxx_Write_Byte(W25X_ManufactDeviceID);      //发送读取ID命令	    
	W25Qxx_Write_Byte(0x00); 	    
	W25Qxx_Write_Byte(0x00); 	    
	W25Qxx_Write_Byte(0x00); 	 			   
	id|=W25Qxx_Read_Byte()<<8;  
	id|=W25Qxx_Read_Byte();	 
	W25Q128_CS_HIGH();
	return id;
} 		

/********************************************************************************
	*方法名称： uint8_t W25Qxx_Read_SR(void) 
	*功能：     读取W25Qxx的状态寄存器
	*参数：     无
	*返回：			状态寄存器值
	寄存器定义
	BIT7  6   5   4   3   2   1   0
	SPR   RV  TB BP2 BP1 BP0 WEL BUSY
	SPR:默认0,状态寄存器保护位,配合WP使用
	TB,BP2,BP1,BP0:FLASH区域写保护设置
	WEL:写使能锁定
	BUSY:忙标记位(1,忙;0,空闲)
	默认:0x00
*********************************************************************************/ 
uint8_t W25Qxx_Read_SR(void)   
{  
	uint8_t byte=0;   
	W25Q128_CS_LOW();                           //使能器件   
	W25Qxx_Write_Byte(W25X_ReadStatusReg1);      //发送读取状态寄存器命令    
	byte=W25Qxx_Read_Byte();                    //读取一个字节  
	W25Q128_CS_HIGH();                           //取消片选     
	return byte;   
} 

/********************************************************************************
	*方法名称： void W25Qxx_Write_SR(uint8_t date) 
	*功能：     写W25QXX状态寄存器
	*参数：     无
	*返回：			无
  只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
*********************************************************************************/ 
void W25Qxx_Write_SR(uint8_t date)   
{   
	W25Q128_CS_LOW();                                //使能器件   
	W25Qxx_Write_Byte(W25X_WriteStatusReg1);         //发送写取状态寄存器命令    
	W25Qxx_Write_Byte(date);               	       //写入一个字节  
	W25Q128_CS_HIGH();                                   //取消片选     	      
} 

/********************************************************************************
	*方法名称： void W25Qxx_Write_Enable(void)  
	*功能：     W25Qxx芯片写使能	， 编程与擦除前需要写使能
	*参数：     无
	*返回：		  无
*********************************************************************************/ 
 void W25Qxx_Write_Enable(void)   
{
	W25Q128_CS_LOW();               
    W25Qxx_Write_Byte(W25X_WriteEnable); 	
	W25Q128_CS_HIGH();                   	    
} 


/********************************************************************************
	*方法名称： void W25Qxx_Write_Disable(void)    
	*功能：     W25Qxx芯片写失能	， 编程与擦除后需要写失能
	*参数：     无
	*返回：		  无
*********************************************************************************/ 
void W25Qxx_Write_Disable(void)   
{  
	W25Q128_CS_LOW();                  
    W25Qxx_Write_Byte(W25X_WriteDisable);  
	W25Q128_CS_HIGH();                    
} 	

/********************************************************************************
	*方法名称： void W25Qxx_Read(uint8_t* buffer,uint32_t addr,uint16_t len)     
	*功能：     按照指定地址（以字节为单位）读取芯片数据，
  *参数：     buffer:数据缓存
              addr  :读取地址；
              len   :读取数据长度
	*返回：		  无
*********************************************************************************/ 
HAL_StatusTypeDef W25Qxx_Read(uint8_t* buffer,uint32_t addr,uint32_t len)   
{			
    HAL_StatusTypeDef  rtn ;	
	W25Q128_CS_LOW();                  
    W25Qxx_Write_Byte(W25X_ReadData);   //发送读取命令   
    W25Qxx_Write_Byte((uint8_t)((addr)>>16));  	//发送24bit地址    
    W25Qxx_Write_Byte((uint8_t)((addr)>>8)) ;   
    W25Qxx_Write_Byte((uint8_t) addr);   
    rtn = W25Qxx_Read_Buffer(buffer,len)    ;
	W25Q128_CS_HIGH();                    
    return rtn;
} 

/********************************************************************************
	*方法名称： void W25Qxx_Wait_Busy(void)      
	*功能：     等待芯片完成操作, 主要用于线程内对芯片完成操作
  *参数：     无       
	*返回：		  无
*********************************************************************************/ 
HAL_StatusTypeDef W25Qxx_Wait_Idle(void)   
{  
  int i = 0 , n ; 	 
	while((W25Qxx_Read_SR()&0x01)==0x01)  // 等待BUSY位清空
	{
		 for ( n = 0; n < 8800 ; n ++) {}
		 if(++ i >= 1000)  break; 
	}	
	if(i < 1000)  return HAL_OK;
  else          return HAL_TIMEOUT;		
} 

/********************************************************************************
	*方法名称： void W25Qxx_Write_Page(uint8_t* buffer,uint32_t addr,uint16_t len)    
	*功能：     按照指定地址（以字节为单位）写数据到芯片，最大256个， 每个扇区需要分多次写入
  *参数：     buffer:数据缓存
              addr  :写地址；
              len   :写数据长度
	*返回：		  无
*********************************************************************************/ 
HAL_StatusTypeDef W25Qxx_Write_Page(uint8_t* buffer,uint32_t addr,uint16_t len)
{ 
	W25Qxx_Write_Enable();                  	        
	W25Q128_CS_LOW();
	W25Qxx_Write_Byte(W25X_PageProgram);      	//发送写页命令   
	W25Qxx_Write_Byte((uint8_t)((addr)>>16)); 	          //发送24bit地址    
	W25Qxx_Write_Byte((uint8_t)((addr)>>8)) ;   
	W25Qxx_Write_Byte((uint8_t)  addr);   
	W25Qxx_Write_Buffer(buffer,len);                     //循环写数
	W25Q128_CS_HIGH();
  return  W25Qxx_Wait_Idle() ;	
} 

/********************************************************************************
	*方法名称： void W25Qxx_Sector_Erase(uint32_t sector)      
	*功能：     按照指定扇区擦除一个扇区( 4096 byte),擦除时间:40ms左右
  *参数：     sector:扇区号       
	*返回：		  无
*********************************************************************************/ 
HAL_StatusTypeDef W25Qxx_Sector_Erase(uint32_t sector)   
{  
 	uint32_t addr  =  W25Qxx_Sector_Size * sector;
    W25Qxx_Write_Enable();                 
    W25Qxx_Wait_Idle();   
  	W25Q128_CS_LOW();
    W25Qxx_Write_Byte(W25X_SectorErase);      	
    W25Qxx_Write_Byte((uint8_t)((addr)>>16));  	
    W25Qxx_Write_Byte((uint8_t)((addr)>>8));   
    W25Qxx_Write_Byte((uint8_t)  addr);  
	W25Q128_CS_HIGH();
   	//等待擦除完成                    	      
    return W25Qxx_Wait_Idle();   
} 

/********************************************************************************
	*方法名称： void W25Qxx_Write(uint8_t* buffer,uint32_t addr,uint16_t len)     
	*功能：     按照指定地址（以字节为单位）写数据到芯片；
  *参数：     buffer:数据缓存
              addr  :写入地址；
              len   :写入数据长度
	*返回：		  无
*********************************************************************************/ 
HAL_StatusTypeDef W25Qxx_Write(uint8_t* buffer,uint32_t addr,uint32_t len)   
{								    
	uint32_t size        ; 
  uint32_t  chipSector , addrAdd , writeCount ,writeAddr ;
	uint16_t i   = 0  ,  l    ;

	//  初步验证输入参数
	if(len == 0 || buffer == NULL || addr > W25QxxInfo.chipSectorNbr*W25QxxInfo.chipSectorSize)   return  HAL_ERROR;
	
	//   连续写入数据
	while(len  >  0)
	{
		
		  l =   addr % W25Qxx_Sector_Size ;
		  //  拆分需要连续写入的字节，分物理扇区依次写入
		  if(((W25Qxx_Sector_Size) - l) <  len)
			{
			   writeCount = ((W25Qxx_Sector_Size) - l);
				 len       -= ((W25Qxx_Sector_Size) - l); 
			}
			else
			{
			   writeCount       = len;
				 len             -= len;
			}
			
			//   计算写入地址所在的物理扇区与地址偏移
			chipSector =addr / 4096; // 芯片扇区地址  
			addrAdd    =addr % 4096; // 在扇区内的偏移
			
			//  计算写入地址
			writeAddr = chipSector  *   4096 ; 
		

			//   读扇区所有数据
			W25Qxx_Read(W25QxxWriteBuffer, chipSector * (W25Qxx_Sector_Size) 
												,W25Qxx_Sector_Size) ;
			
			 
			 //   将带写入的数据拷贝到指定位置
			 memcpy(W25QxxWriteBuffer + addrAdd , buffer , writeCount);

			 //  擦除对应扇区
			 W25Qxx_Sector_Erase(chipSector);  
			 
			 //  分16页写入
			 for(i=0; i <  16 ; i++)
				{
					 size  = (i * 256)                                               ; 
					 W25Qxx_Write_Page(W25QxxWriteBuffer+size,writeAddr+size,256) ;
				}
			W25Qxx_Read(W25QxxWriteBuffer,addr,writeCount)       ;
				
			if(memcmp(W25QxxWriteBuffer ,buffer,writeCount) != 0)
			{
				return  HAL_ERROR;				
			}
			else
			{
				//WS_Debug(0,"W25Qxx write OK Sector:%d~%d\r\n",writeAddr,writeAddr+writeCount);
			}
			
			addr   += writeCount;   //  添加地址偏移，完成部分写入
			buffer += writeCount;
			
	}
  return HAL_OK;	     		    	      
}

/********************************************************************************
	*方法名称： HAL_StatusTypeDef  W25Qxx_Sector_Write(uint8_t *buffer  ,uint32_t sector ,  uint32_t size)       
	*功能：     按照指定扇区写入len个扇区( len * 4096 byte),写入时间:单个扇区写入70ms左右
  *参数：     buffer：缓存地址；  sector:扇区号 ；len: 扇区数      
	*返回：		  操作状态
*********************************************************************************/ 
HAL_StatusTypeDef  W25Qxx_Sector_Write(uint8_t *buffer  ,uint32_t sector ,  uint32_t len)   
{
	   return  W25Qxx_Write(buffer,sector*W25Qxx_Sector_Size,len * W25Qxx_Sector_Size);    
}

/********************************************************************************
	*方法名称： HAL_StatusTypeDef  W25Qxx_Sector_Read(uint8_t *buffer  ,uint32_t sector ,  uint32_t size)       
	*功能：     按照指定扇区写入len个扇区( len * 4096 byte)
  *参数：     buffer：缓存地址；  sector:扇区号 ；len: 扇区数      
	*返回：		  操作状态
*********************************************************************************/ 
HAL_StatusTypeDef  W25Qxx_Sector_Read(uint8_t *buffer  ,uint32_t sector ,  uint32_t len) 
{
     return  W25Qxx_Read (buffer,sector * W25Qxx_Sector_Size , len * W25Qxx_Sector_Size) ;
}

//    读写测试
void  W25Qxx_Write_Read_Check(uint32_t addr)
{
      uint16_t i , ok = 0;
	    
	    static  uint8_t buf[4096];  
	    for (i = 0; i < sizeof(buf) ; i ++)
	    {
			   buf[i] = i;
			}
			W25Qxx_Write(buf,addr,sizeof(buf));
	    for (i = 0; i < sizeof(buf) ; i ++)
	    {
			   buf[i] = 55;
			}
			W25Qxx_Read(buf,addr,sizeof(buf));
			for (i = 0; i < sizeof(buf) ; i ++)
	    {
			  if(  buf[i] != i%256)
				{
					 ok = 1;
				}
			}
			if(ok == 0)
			{

			}
}




