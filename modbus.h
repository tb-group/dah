#include <termios.h>
#include <unistd.h>

typedef unsigned char             UINT8;
typedef signed char               INT8;
typedef char                      CHAR;
typedef unsigned short            UINT16;
typedef signed short              INT16;
typedef unsigned int              UINT32;
typedef	signed int                INT32;
typedef unsigned long int         UINT64;
typedef unsigned long long int    LP64;
typedef signed long int           INT64;
typedef unsigned long             ULONG;
typedef signed long               SLONG;
typedef UINT8                     BOOLEAN;
typedef void                      VOID;

typedef UINT32         OPSTAT;
#define SUCCESS        0
#define FAILURE        -1
#define true   			1
#define false			0

#define 	TRUE 		1
#define 	FALSE 		0

#define  	TB_INVALID_U32					0xffffffff
#define  	TB_INVALID_U16					0xffff
#define  	TB_INVALID_U8					0xff
//#define HCU_SPS_COM_PORT_PATH_0  "/dev/ttyAMA0"
#define HCU_SPS_COM_PORT_PATH_0  "/dev/ttyUSB0"
#define HCU_SPS_COM_PORT_PATH_1  "/dev/ttyO1"
#define HCU_SPS_COM_PORT_PATH_2  "/dev/ttyO2"
#define HCU_SPS_COM_PORT_PATH_3  "/dev/ttyO3"
#define HCU_SPS_COM_PORT_PATH_4  "/dev/ttyO4"

#define COM_PORT_0 	0x00
#define COM_PORT_1 	0x01
#define COM_PORT_2 	0x02
#define COM_PORT_3 	0x03
#define COM_PORT_4 	0x04
#define COM_PORT_5 	0x05
#define COM_PORT_6 	0x06
#define COM_PORT_7 	0x07

typedef enum
{
	L3PO_temp_min = 0,
	L3PO_temp_none = 0,
	L3PO_temp_data_req = 0x01, //Data Request
	L3PO_temp_set_switch = 0x02,
	L3PO_temp_set_modbus_address =0x03,
	L3PO_temp_set_work_cycle = 0x04, //In second
	L3PO_temp_set_sample_cycle = 0x05, //In second
	L3PO_temp_set_sample_number = 0x06,
	L3PO_temp_data_report = 0x81, //Data Report
	L3PO_temp_read_switch = 0x82,
	L3PO_temp_read_modbus_address = 0x83,
	L3PO_temp_read_work_cycle = 0x84, //In second
	L3PO_temp_read_sample_cycle = 0x85, //In second
	L3PO_temp_read_sample_number = 0x86,
	L3PO_temp_max,
}L3TempOptIdDef;
typedef enum
{
	L3PO_humid_min = 0,
	L3PO_humid_none = 0,
	L3PO_humid_data_req = 0x01, //Data Request
	L3PO_humid_set_switch = 0x02,
	L3PO_humid_set_modbus_address =0x03,
	L3PO_humid_set_work_cycle = 0x04, //In second
	L3PO_humid_set_sample_cycle = 0x05, //In second
	L3PO_humid_set_sample_number = 0x06,
	L3PO_humid_data_report = 0x81, //Data Report
	L3PO_humid_read_switch = 0x82,
	L3PO_humid_read_modbus_address = 0x83,
	L3PO_humid_read_work_cycle = 0x84, //In second
	L3PO_humid_read_sample_cycle = 0x85, //In second
	L3PO_humid_read_sample_number = 0x86,
	L3PO_humid_max,
}L3HumidOptIdDef;

typedef enum
{
	L3PO_noise_min = 0,
	L3PO_noise_none = 0,
	L3PO_noise_data_req = 0x01, //Data Request
	L3PO_noise_set_switch = 0x02,
	L3PO_noise_set_modbus_address =0x03,
	L3PO_noise_set_work_cycle = 0x04, //In second
	L3PO_noise_set_sample_cycle = 0x05, //In second
	L3PO_noise_set_sample_number = 0x06,
	L3PO_noise_data_report = 0x81, //Data Report
	L3PO_noise_read_switch = 0x82,
	L3PO_noise_read_modbus_address = 0x83,
	L3PO_noise_read_work_cycle = 0x84, //In second
	L3PO_noise_read_sample_cycle = 0x85, //In second
	L3PO_noise_read_sample_number = 0x86,
	L3PO_noise_max,
}L3NoiseOptIdDef;

typedef enum
{
	TEMP_REG_DATA_READ = 0,  //2B
	TEMP_LENGTH_OF_REG = 0x02, //2个寄存器，返回4B
	TEMP_MODBUS_GENERIC_FUNC_DATA_INQUERY = 0x03,
}TempRegisterSelfDef;


typedef enum
{
	NOISE_REG_DATA_READ = 0,  //2B
	NOISE_LENGTH_OF_REG = 0x02, //2个寄存器，返回4B
	NOISE_CRC16_PRESENT = FALSE, //CRC16不存在，取值TRUE/FALSE
	NOISE_FUNCTION_CODE = 0x57,
	NOISE_MODBUS_GENERIC_FUNC_DATA_INQUERY = 0x04,
}NoiseRegisterSelfDef;


typedef enum
{
	MODBUS_NONE_RTU_EQUIPMENT_ID = 0,
	MODBUS_PM25_RTU_EQUIPMENT_ID = 0x1,
	MODBUS_WINDSPD_RTU_EQUIPMENT_ID = 0x02,
	MODBUS_WINDDIR_RTU_EQUIPMENT_ID = 0x03,
	MODBUS_EMC_RTU_EQUIPMENT_ID = 0x05,
	MODBUS_TEMP_RTU_EQUIPMENT_ID = 0x06,
	MODBUS_HUMID_RTU_EQUIPMENT_ID = 0x06,
	MODBUS_INVALID_RTU_EQUIPMENT_ID = 0xFF,
}ModbusSensorEquipmentIdDef;
//功能码定义
#define MODBUS_GENERIC_FUNC_DATA_INQUERY  0x03
#define MODBUS_PM25_FUNC_DATA_INQUERY  0x03

typedef struct SerialPort
{
	UINT8 id;			/* COM1=>0, COM1=>1, COM2=>2 ....  */
	UINT32 nSpeed;		/* 1200, 2400, 4800, 4800, 9600, 19200, 38400, 57600, 115200 */
	UINT16 nBits;		/* 7 or 8 */
	UINT8 nEvent;		/* '0', 'N', 'E' */
	UINT16 nStop;		/* 0, or 1 */
	UINT32 fd;			/* file descriptor for com port */
	UINT8 vTime;        /* */
	UINT8 vMin;         /* */
	UINT32 c_lflag;     /* ICANON : enable canonical input */
} SerialPort_t;

typedef struct SerialPortCom
{
	UINT32 id;			/* COM1=>0, COM1=>1, COM2=>2 ....  */
	UINT32 nSpeed;		/* 1200, 2400, 4800, 4800, 9600, 19200, 38400, 57600, 115200 */
	UINT16 nBits;		/* 7 or 8 */
	UINT8 nEvent;		/* '0', 'N', 'E' */
	UINT16 nStop;		/* 0, or 1 */
	UINT32 fd;			/* file descriptor for com port */
	UINT8 vTime;        /* */
	UINT8 vMin;         /* */
	UINT32 c_lflag;     /* ICANON : enable canonical input */
}SerialPortCom_t;


typedef struct SerialModbusMsgBuf
{
	UINT32 curLen;
	UINT8  curBuf[1500];
}SerialModbusMsgBuf_t;


typedef struct  msg_struct_temp_modbus_data_read //
{
	UINT8  cmdId;
	UINT8  optId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT32 equId;
	UINT32 length;
}msg_struct_temp_modbus_data_read_t;


extern UINT32 modbus_sps485_serial_port_get(SerialPortCom_t *sp, UINT8 *send_buf, UINT32 Len);
UINT32 modbus_spsapi_serial_port_send(SerialPortCom_t *sp, UINT8 *send_buf, UINT32 Len);
UINT16 CalcCRCModBus(UINT8 cDataIn, UINT16 wCRCIn);
