#include "modbus.h"
#include <sys/types.h>
#include <sys/stat.h>    
#include <fcntl.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
// support function

UINT32 SerialPortSetVtimeVmin(SerialPort_t *sp, UINT8 vTime, UINT8 vMin)
{
	/* Local variable */
	struct termios oldtio;
	UINT8 fd = sp->fd;

	if ( tcgetattr(fd, &oldtio) !=  SUCCESS )
	{
		printf("SPS485: Com port control fd is not valid\n");
		return FAILURE;
	}

	/* Set VTIME and VMIN */
	oldtio.c_cc[VTIME]  = vTime;
	oldtio.c_cc[VMIN] = vMin;  /* If there is one in the buffer, it will be return from block read, */

	/* Clean the buffer !!! before !!! the parameter active */
	tcflush(fd, TCIFLUSH);

	/* Set parameters */
	if((tcsetattr(fd, TCSANOW, &oldtio))!=0)
	{
		printf("SPS485: COM port set error\n");
		return FAILURE;
	}

	/* Save VTIME and VMIN */
	sp->vTime = vTime;
	sp->vMin = vMin;

	return SUCCESS;
}



UINT16 CalcCRCModBus(UINT8 cDataIn, UINT16 wCRCIn)
{
    UINT16 wCheck = 0;
    wCRCIn = wCRCIn ^ cDataIn;
    int i=0;

    for(i=0;i<8;i++)
    {
        wCheck = wCRCIn & 1;
        wCRCIn = wCRCIn >> 1;
        wCRCIn = wCRCIn & 0x7fff;

        if(wCheck == 1)
        {
            wCRCIn = wCRCIn ^ 0xa001;
        }
        wCRCIn = wCRCIn & 0xffff;
    }

    return wCRCIn;
}


void CheckCRCModBus(const UINT8* pDataIn, int iLenIn, UINT16* pCRCOut)
{
	UINT16 wHi = 0;
	UINT16 wLo = 0;
	UINT16 wCRC;
    wCRC = 0xFFFF;
    int i=0;

    for (i=0;i<iLenIn;i++)
    {
        wCRC = CalcCRCModBus(*pDataIn, wCRC);
        pDataIn++;
    }

    wHi = wCRC / 256;
    wLo = wCRC % 256;
    wCRC = (wHi << 8) | wLo;

    *pCRCOut = wCRC;
}



OPSTAT func_modbus_temp_msg_pack(msg_struct_temp_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_temp_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((TEMP_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(TEMP_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((TEMP_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(TEMP_LENGTH_OF_REG & 0x0FF); //长度低位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	default:
		printf("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;
}

OPSTAT func_modbus_noise_msg_pack(msg_struct_noise_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF){
		zHcuSysStaPm.taskRunErrCnt[TASK_ID_MODBUS]++;
		return FAILURE;
	}
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(NOISE_MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_noise_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((NOISE_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(NOISE_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((NOISE_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(NOISE_LENGTH_OF_REG & 0x0FF); //长度低位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		zHcuSysStaPm.taskRunErrCnt[TASK_ID_MODBUS]++;
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;

OPSTAT func_modbus_temp_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_temp_modbus_data_read_t *rcv)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>1500)){
		printf("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		printf("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		printf("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		printf("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_temp_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != TEMP_LENGTH_OF_REG *2){
			printf("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		t0 = buf->curBuf[index++];  //前两个数据是温度，后面的2个数据是湿度
		t1 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
//		snd->tempValue = t0 + t1;
		break;

	default:
		printf("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}


UINT32 hcu_spsapi_serial_port_send(SerialPortCom_t *sp, UINT8 *send_buf, UINT32 Len)
{
     UINT16  fd = sp->fd;
     UINT32 ret = SUCCESS;
     printf("MODBUS: send data content: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",send_buf[0],send_buf[1],send_buf[2],send_buf[3],send_buf[4],send_buf[5],send_buf[6],send_buf[7],send_buf[8]);
     ret = write(fd, send_buf, Len);
     if (-1 == ret)
     {
         printf("Modbus: Write device error\n");
         return -1;
     }
     return ret;
}


UINT32 spsapi_SerialPortSet(SerialPort_t *sp)
{
	/* Local variable */
	struct termios newtio, oldtio;
	UINT32 cflag, iflag, oflag, lflag;

	if ( tcgetattr(sp->fd, &oldtio)  !=  SUCCESS)
	{
		printf("SPSAPI: Com port control fd is not valid\n");
		return FAILURE;
	}

	/* Set all bit to Zero */
	bzero( &newtio, sizeof( newtio ) );

	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = sp->c_lflag;

	switch( sp->nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	default:
		printf("SPSAPI: Invalid bit lenth set for com port, nBit should be 7 or 8, error return\n");
		return FAILURE;
	}

	switch( sp->nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	default:
		printf("SPSAPI: Invalid parity for com port, nEvent should be '0', 'E', or 'N', error return\n");
		return FAILURE;
	}

	switch( sp->nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 19200:
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
	case 38400:
		cfsetispeed(&newtio, B38400);
		cfsetospeed(&newtio, B38400);
		break;
	case 57600:
		cfsetispeed(&newtio, B57600);
		cfsetospeed(&newtio, B57600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 230400:
		cfsetispeed(&newtio, B230400);
		cfsetospeed(&newtio, B230400);
		break;
	default:
		printf("SPSAPI: Invalid speed set for com port, Error return\n");
		return FAILURE;
		break;
	}

	if( 1 == sp->nStop)
		newtio.c_cflag &=  ~CSTOPB;
	else if ( 2 == sp->nStop)
		newtio.c_cflag |=  CSTOPB;
	else
	{
		printf("SPSAPI: Invalid stop bit for com port, Error return\n");
		return FAILURE;
	}

	/* default VTIME and VMIN */
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;  /* If there is one in the buffer, it will be return from block read, */

	/* Clean the buffer !!! before !!! the parameter active */
	tcflush(sp->fd, TCIFLUSH);

	/* Set parameters */
	if((tcsetattr(sp->fd,TCSANOW, &newtio))!=0)
	{
		printf("SPS485: COM port set error\n");
		return FAILURE;
	}

	/* Save VTIME and VMIN */
	sp->vTime = 0;
	sp->vMin = 1;

	/* Read out the config paramter for debug */
	cflag = (UINT32)newtio.c_cflag;
	iflag = (UINT32)newtio.c_iflag;
	oflag = (UINT32)newtio.c_oflag;
	lflag = (UINT32)newtio.c_lflag;

	
		printf("SPSAPI: COM port flags: c_cflag = 0x%X, c_iflag = 0x%X, c_oflag = 0x%X, c_lflag = 0x%X, VTIME = 0x%d, TMIN = 0x%d\n", cflag, iflag, oflag, lflag, newtio.c_cc[VTIME], newtio.c_cc[VMIN] = 1);
		printf("SPSAPI: COM port set done!\n");


	return SUCCESS;
}

UINT32 modbus_sps485_serial_init(SerialPort_t *sp)
{
	UINT32 ret = SUCCESS;
  printf(" init fd is %d, and the port is %d ", sp->fd, HCU_SPS_COM_PORT_PATH_0);
	sp->fd = open(HCU_SPS_COM_PORT_PATH_0, O_RDWR|O_NOCTTY|O_NDELAY);
	printf("open the modbus  fd= %d \n",sp->fd );
	
	ret = spsapi_SerialPortSet(sp);
	return ret; 
	
}

UINT32 modbus_serial_port_send(SerialPort_t *sp, UINT8 *send_buf, UINT32 Len)
{
     UINT16  fd = sp->fd;
     UINT32 ret = SUCCESS;
 
     printf("MODBUS: length is%d, and send data content: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",Len, send_buf[0],send_buf[1],send_buf[2],send_buf[3],send_buf[4],send_buf[5],send_buf[6],send_buf[7],send_buf[8]);
 
     ret = write(fd, send_buf, Len);
     if (FAILURE == ret)
     {
      printf("SPSAPI: Write device error\n");
     }
}     
        
 
/*
UINT32 modbus_spsapi_serial_port_get(SerialPort_t *sp, UINT8 *rcv_buf, UINT32 Len)
{
	UINT16 fd = sp->fd;
	UINT32 ret = SUCCESS;
	ret = read(fd, rcv_buf, Len);
			if (FAILURE == ret)
		   {
				printf("SPSAPI: Read Data failure \n");
				return FAILURE;
			 }
    printf("MODBUS: Received Temp data length: %d \n ", ret);
	  printf("MODBUS: Received Temp data content: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",rcv_buf[0],rcv_buf[1],rcv_buf[2],rcv_buf[3],rcv_buf[4],rcv_buf[5],rcv_buf[6],rcv_buf[7],rcv_buf[8]);
}
*/

UINT32 modbus_spsapi_serial_port_get(SerialPort_t *sp, UINT8 *rcv_buf, UINT32 Len)
{
	UINT16 fd = sp->fd;
	UINT32 ret = SUCCESS;

	struct timeval tv;
	tv.tv_sec = 3;
	tv.tv_usec = 0;
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	if(select(1+fd, &rfds, NULL, NULL, &tv) > 0)
	{
		if(FD_ISSET(fd, &rfds))
		{
			ret = read(fd, rcv_buf, Len);
			//HcuDebugPrint("SPSAPI: Len of read %d\n", ret);
			if (FAILURE == ret)
		   {
				printf("SPSAPI: Read Data failure \n");
				return FAILURE;
			}
			else if(!ret)
			{
				 printf("SPSAPI: Read return 0 byte \n");
				 return FAILURE;
			 }

			return ret;
		}

		else
		{
			printf("SPSAPI: Read Data failure \n");
			return  FAILURE;
		}
	}

	else
	{
		printf("SPSAPI: Read Data failure \n");
		return  FAILURE;

	}
}

UINT32 modbus_serial_port_close(UINT32 fd)
{
	close(fd);
}

main()
{
  
  //  {mosquitto_pub -h localhost -p 1883 -t "sensors" -m '{"serialNumber":"SN-001", "model":"T1000", "temperature":36.6}'}
  UINT32 temp;
  // define global variable and inititalization
   UINT32 ret = SUCCESS;
   msg_struct_temp_modbus_data_read_t rev_buf;
   SerialPort_t gSerialPort = {COM_PORT_3, 9600, 8, 'N', 1, TB_INVALID_U16, 0, 1, 0};
   SerialModbusMsgBuf_t currentModbusBuf;
   msg_struct_temp_modbus_data_read_t rcv;
	 memset(&rcv, 0, sizeof(msg_struct_temp_modbus_data_read_t)); 
      
   ret=modbus_sps485_serial_init(&gSerialPort);
   if (FAILURE == ret)
	{
		printf("SPSAPI: Open Serial Port %d failure.\n");
		exit(0);
	}
   
   SerialPortSetVtimeVmin(&gSerialPort, 0, 9);
  // 1. for temperature and humunity
  //1.1  pack message includr CRC generate
 	 memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	 rcv.optId = L3PO_temp_data_req;
	 rcv.equId = 6;	 
	 ret = func_modbus_temp_msg_pack(&rcv, &currentModbusBuf);
   if (ret == FAILURE){
	 printf("MODBUS: Error pack message!\n");
	 exit(0);
	 }
 
   // 1.2 send out the request message . 
   ret = modbus_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);
   printf("send out the message, ret = %d \n", ret);
    
  if (FAILURE == ret)
	{
		printf("SPSAPI: serial port send fail.\n");
		modbus_serial_port_close(gSerialPort.fd);
		exit(0);
	}
   
   usleep(50);  
   // recevie 
   memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
   modbus_spsapi_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, 9);
   
   
//   msg_struct_modbus_temp_data_report_t snd;
//	memset(&snd, 0, sizeof(msg_struct_modbus_temp_data_report_t));

//   func_modbus_temp_msg_unpack(&currentModbusBuf, &rcv);   
  
	 printf("MODBUS: Received Temp data content: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7],currentModbusBuf.curBuf[8]);
   temp = (currentModbusBuf.curBuf[3]*256+currentModbusBuf.curBuf[4])/10;
   printf("temperature is %02X\n", temp);

   char cmd[400];
   sprintf(cmd,"mosquitto_pub -h localhost -p 1883 -t \"sensors\" -m '{\"serialNumber\":\"SH067\", \"model\":\"T1000\", \"temperature\":%02d}\'",temp);

 //  sprintf(cmd,  "./mod_bus.sh %02X", temp);
   system(cmd);
   printf("send publish, and send cmd is %s\n",cmd);
   
 // for noise
 
 // 2. for noise.   
   memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	 rcv.optId = L3PO_temp_data_req;
	 rcv.equId = 7;	 
	 ret = func_modbus_noise_msg_pack(&rcv, &currentModbusBuf);
   if (ret == FAILURE){
	 printf("MODBUS: Error pack message!\n");
	 exit(0);
	 }
 
   // 1.2 send out the request message . 
   ret = modbus_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);
   printf("send out the message, ret = %d \n", ret);
    
  if (FAILURE == ret)
	{
		printf("SPSAPI: serial port send fail.\n");
		modbus_serial_port_close(gSerialPort.fd);
		exit(0);
	}
   
   usleep(50);  
   // recevie 
   memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
   modbus_spsapi_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, 1000);
   
   
//   msg_struct_modbus_temp_data_report_t snd;
//	memset(&snd, 0, sizeof(msg_struct_modbus_temp_data_report_t));

//   func_modbus_temp_msg_unpack(&currentModbusBuf, &rcv);   
  
	 printf("MODBUS: Received Noise data content: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7],currentModbusBuf.curBuf[8]);
   temp = (currentModbusBuf.curBuf[3]*256+currentModbusBuf.curBuf[4])/10;
   printf("temperature is %02X\n", temp);

   char cmd[400];
   sprintf(cmd,"mosquitto_pub -h localhost -p 1883 -t \"sensors\" -m '{\"serialNumber\":\"SH067\", \"model\":\"T1000\", \"noise\":%02d}\'",temp);

 //  sprintf(cmd,  "./mod_bus.sh %02X", temp);
   system(cmd);
   printf("send Noise publish, and send cmd is %s\n",cmd);
    
   // clode the port 
   modbus_serial_port_close(gSerialPort.fd);

   return 0;

}
