#include "Energomera_CE301.h"
#include "Debug.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_uart.h"

#ifndef ENERGOMERA_PORT
#define ENERGOMERA_PORT HAL_UART_PORT_1
#endif

#define ENERGOMERA_CE301_CV_RESPONSE_LENGTH 15
#define ENERGOMERA_CE301_E_RESPONSE_LENGTH 19
#define ENERGOMERA_CE301_READY_RESPONSE_LENGTH 4
#define ENERGOMERA_CE301_REQUEST_LENGTH 6

static void Energomera_CE301_StartStopData(uint8 serial_num, uint8 cmd);
static bool Energomera_CE301_CheckReady(void);

static void Energomera_CE301_RequestMeasure(uint8 serial_num, uint8 cmd);

static current_values_t Energomera_CE301_ReadCurrentValues(uint8 cmd);

static uint32 Energomera_CE301_ReadEnergy(uint8 cmd);

static uint16 MODBUS_CRC16( const unsigned char *buf, unsigned int len );

extern zclEnergomera_t Energomera_CE301_dev = {&Energomera_CE301_StartStopData, &Energomera_CE301_CheckReady, &Energomera_CE301_RequestMeasure, &Energomera_CE301_ReadCurrentValues, &Energomera_CE301_ReadEnergy};



static void Energomera_CE301_StartStopData(uint8 serial_num, uint8 cmd)
{
  if (cmd == 1) {
    uint8 readEnergomera[11]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
    readEnergomera[0] = serial_num;
    readEnergomera[1] = cmd; 
    readEnergomera[2] = 0x01; // уровень доступа
    
    // пароль
    for (int i = 3; (i <= 9); i++) 
    {
      readEnergomera[i] = 0x01;
    }
    
    uint16 crc = MODBUS_CRC16(readEnergomera, 9);

    readEnergomera[9] = crc & 0xFF;
    readEnergomera[10] = (crc>>8) & 0xFF;

    HalUARTWrite(ENERGOMERA_PORT, readEnergomera, sizeof(readEnergomera) / sizeof(readEnergomera[0])); 
    
    LREP("Energomera sent: ");
    for (int i = 0; (i < sizeof(readEnergomera) / sizeof(readEnergomera[0])); i++) 
    {
      LREP("0x%X ", readEnergomera[i]);
    }
    LREP("\r\n");
  }
  else {
    uint8 readEnergomera[4]  = {0x00, 0x00, 0x00, 0x00};
  
    readEnergomera[0] = serial_num;
    readEnergomera[1] = cmd; 
    
    uint16 crc = MODBUS_CRC16(readEnergomera, 2);

    readEnergomera[2] = crc & 0xFF;
    readEnergomera[3] = (crc>>8) & 0xFF;

    HalUARTWrite(ENERGOMERA_PORT, readEnergomera, sizeof(readEnergomera) / sizeof(readEnergomera[0])); 
    
    LREP("Energomera sent: ");
    for (int i = 0; (i < sizeof(readEnergomera) / sizeof(readEnergomera[0])); i++) 
    {
      LREP("0x%X ", readEnergomera[i]);
    }
    LREP("\r\n");
  }
  
}


static bool Energomera_CE301_CheckReady()
{
  uint8 response[ENERGOMERA_CE301_READY_RESPONSE_LENGTH] = {0x00, 0x00, 0x00, 0x00};

  HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));

  LREP("Energomera received: ");
  for (int i = 0; i <= ENERGOMERA_CE301_READY_RESPONSE_LENGTH - 1; i++) 
  {
    LREP("0x%X ", response[i]);
  }
  LREP("\r\n");

  uint16 crc = MODBUS_CRC16(response, ENERGOMERA_CE301_READY_RESPONSE_LENGTH - 2);

  LREP("Real CRC: ");
  LREP("0x%X ", crc & 0xFF);
  LREP("0x%X\r\n", (crc>>8) & 0xFF);
  
  if (response[ENERGOMERA_CE301_READY_RESPONSE_LENGTH - 2] != (crc & 0xFF) || response[ENERGOMERA_CE301_READY_RESPONSE_LENGTH - 1] != ((crc>>8) & 0xFF)) {
    LREPMaster("Invalid response\r\n");
    HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
    return FALSE;
  }
  else if (response[1] != 0) {
    LREPMaster("Data error\r\n");
    HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
    return FALSE;
  }
  
  return TRUE;
}


void Energomera_CE301_RequestMeasure(uint8 serial_num, uint8 cmd) 
{
  uint8 readEnergomera[ENERGOMERA_CE301_REQUEST_LENGTH]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  readEnergomera[0] = serial_num;
  readEnergomera[3] = cmd;
  if ((cmd > 0x00) & (cmd < 0x05)) {
    readEnergomera[1] = 0x05;
    readEnergomera[2] = 0x00;
  }
  else {
    readEnergomera[1] = 0x08;
    readEnergomera[2] = 0x16;
  }
  
  uint16 crc = MODBUS_CRC16(readEnergomera, 4);
  
  readEnergomera[4] = crc & 0xFF;
  readEnergomera[5] = (crc>>8) & 0xFF;

  HalUARTWrite(ENERGOMERA_PORT, readEnergomera, sizeof(readEnergomera) / sizeof(readEnergomera[0])); 
  
  LREP("Energomera sent: ");
  for (int i = 0; (i < sizeof(readEnergomera) / sizeof(readEnergomera[0])); i++) 
  {
    LREP("0x%X ", readEnergomera[i]);
  }
  LREP("\r\n");
}


current_values_t Energomera_CE301_ReadCurrentValues(uint8 cmd) 
{
  
  current_values_t result = {
    {ENERGOMERA_INVALID_RESPONSE, ENERGOMERA_INVALID_RESPONSE, ENERGOMERA_INVALID_RESPONSE}, 
    {ENERGOMERA_INVALID_RESPONSE, ENERGOMERA_INVALID_RESPONSE, ENERGOMERA_INVALID_RESPONSE}, 
    {ENERGOMERA_INVALID_RESPONSE, ENERGOMERA_INVALID_RESPONSE, ENERGOMERA_INVALID_RESPONSE}
  };
  
  uint8 length;
  uint8 shift;
  
  uint8 response[ENERGOMERA_CE301_CV_RESPONSE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));

  LREP("Energomera received: ");
  for (int i = 0; i <= ENERGOMERA_CE301_CV_RESPONSE_LENGTH - 1; i++) 
  {
    LREP("0x%X ", response[i]);
  }
  LREP("\r\n");
  
  if (cmd == REQ_POWER) {
    length = ENERGOMERA_CE301_CV_RESPONSE_LENGTH;
    shift = 3;
  }
  else {
    length = ENERGOMERA_CE301_CV_RESPONSE_LENGTH - 3;
    shift = 0;
  };
    
  
  uint16 crc = MODBUS_CRC16(response, length - 2);
  
  LREP("Real CRC: ");
  LREP("0x%X ", crc & 0xFF);
  LREP("0x%X\r\n", (crc>>8) & 0xFF);
  
  if (response[length - 2] != (crc & 0xFF) || response[length - 1] != ((crc>>8) & 0xFF)) {
    LREPMaster("Invalid response\r\n");
    HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
    return result;
  }

  switch (cmd) {  
  case REQ_VOLTAGE:
    for (int i = 0; i <= 2; i++) 
      result.Voltage[i] = response[1 + i * 3 + shift] * 0x10000 + response[3 + i * 3 + shift] * 0x100 + response[2 + i * 3 + shift];
    break;
  case REQ_CURRENT:
    for (int i = 0; i <= 2; i++) 
      result.Current[i] = response[1 + i * 3 + shift] * 0x10000 + response[3 + i * 3 + shift] * 0x100 + response[2 + i * 3 + shift];
    break;
  case REQ_POWER:
    for (int i = 0; i <= 2; i++) {
      uint32 power = (uint32)(response[1 + i * 3 + shift] & 0x0f) * 0x10000 + (uint32)response[3 + i * 3 + shift] * 0x100 + response[2 + i * 3 + shift];
      LREP("power %ld\r\n", power);
      result.Power[i] = (int16)(power / 100);
    }
  }
  return result;
}

uint32 Energomera_CE301_ReadEnergy(uint8 cmd) 
{
    uint32 result = ENERGOMERA_INVALID_RESPONSE;
    
    uint8 response[ENERGOMERA_CE301_E_RESPONSE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));

    LREP("Energomera received: ");
    for (int i = 0; i <= ENERGOMERA_CE301_E_RESPONSE_LENGTH - 1; i++) 
    {
      LREP("0x%X ", response[i]);
    }
    LREP("\r\n");
    
    uint16 crc = MODBUS_CRC16(response, ENERGOMERA_CE301_E_RESPONSE_LENGTH - 2);

    LREP("Real CRC: ");
    LREP("0x%X ", crc & 0xFF);
    LREP("0x%X\r\n", (crc>>8) & 0xFF);
    
    if (response[ENERGOMERA_CE301_E_RESPONSE_LENGTH - 2] != (crc & 0xFF) || response[ENERGOMERA_CE301_E_RESPONSE_LENGTH - 1] != ((crc>>8) & 0xFF)) {
        LREPMaster("Invalid response\r\n");
        HalUARTRead(ENERGOMERA_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
        return result;
    }

    result =  (uint32)response[2] * 0x1000000 + (uint32)response[1] * 0x10000 + (uint32)response[4] * 0x100 + (uint32)response[3];
    LREP("Result: %ld\r\n", result);

    return result;
}

static uint16 MODBUS_CRC16 ( const unsigned char *buf, unsigned int len )
{
	uint16 crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			if( crc & 0x0001 )
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	return crc;
}

