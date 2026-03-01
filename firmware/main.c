/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "stdbool.h"
#include "string.h"
#include "bootloader.h"
#include "main.h"

#define BOOTLOADER_VERSION_NUM ((FIRMWARE_VERSION_MAJOR * 10) + FIRMWARE_VERSION_MINOR)

/* deviceId: "GD" + MCU prefix (S=WCH/CH32V) + current (08A/10A) + "12S" + variant (S/M) [+ version (V1/...)]
 * Example: GDS08A12SS, GDS10A12SS, GDS08A12SM, GDS08A12SSV1 */
#define DEVICE_ID "GD" ESC_MCU_PREFIX_STR ESC_CURRENT_STR "12S" ESC_VARIANT_STR ESC_VERSION_STR "\x20\x00"

/* Addresses come from the linker script (esc_ch32v20x_D6.ld / ch32v20x_D6.ld):
 * _flash_start_addr    = 0x08000000  (physical XIP bus alias, same for all CH32V20x)
 * _firmware_start_addr = ADDR(.firmware_start)  (e.g. 0x1000)
 * _eeprom_start_addr   = ADDR(.eeprom)           (e.g. 0xF000)
 * FLASH ORIGIN = 0x00000000, so firmware/eeprom values are offsets from _flash_start_addr. */
extern char _flash_start_addr;
extern char _flash_size;
extern char _firmware_start_addr;
extern char _eeprom_start_addr;

#define CH32_FLASH_START        ((uint32_t)&_flash_start_addr)
#define FIRMWARE_RELATIVE_START ((uint32_t)&_firmware_start_addr)
#define EEPROM_RELATIVE_START   ((uint32_t)&_eeprom_start_addr)
#define FLASH_SIZE              ((uint32_t)&_flash_size)

hardwareVersion_t __attribute__ ((section(".device_type"))) device_type = {
        .deviceId = DEVICE_ID
};

// version_t __attribute__ ((section(".bootloader_info"))) bootloader_version = {
//     .major = FIRMWARE_VERSION_MAJOR,
//     .minor = FIRMWARE_VERSION_MINOR,
// };

typedef void (*pFunction)(void);

#define APPLICATION_ADDRESS  (uint32_t)(CH32_FLASH_START + FIRMWARE_RELATIVE_START)
#define EEPROM_START_ADD     (uint32_t)(CH32_FLASH_START + EEPROM_RELATIVE_START)


#define CMD_RUN              0x00
#define CMD_PROG_FLASH       0x01
#define CMD_ERASE_FLASH      0x02
#define CMD_READ_FLASH_SIL   0x03
#define CMD_VERIFY_FLASH     0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM      0x04
#define CMD_PROG_EEPROM      0x05
#define CMD_READ_SRAM        0x06
#define CMD_READ_FLASH_ATM   0x07
#define CMD_KEEP_ALIVE       0xFD
#define CMD_SET_ADDRESS      0xFF
#define CMD_SET_BUFFER       0xFE

#define input_pin        GPIO_Pin_0
#define input_port       GPIOA
#define PIN_NUMBER       0
#define PORT_LETTER      1



#define  TIME_FACTOR     (12)


uint16_t low_pin_count = 0;
char receviedByte;
int count = 0;
char messagereceived = 0;
uint16_t invalid_command = 0;
uint16_t address_expected_increment;
int cmd = 0;
char eeprom_req = 0;
uint8_t pin_code = PORT_LETTER << 4 | PIN_NUMBER;

/* deviceInfo bytes [4] and [5]: MCU-specific flash page count and bootloader type.
 * CH32V (WCH): 0x1F (31 pages), 0x06
 * Other (e.g. ARM): TBD */
#if defined(CH32V)
#  define DEVICE_INFO_FLASH_PAGES  0x1F
#  define DEVICE_INFO_BL_TYPE      0x06
#else
#  define DEVICE_INFO_FLASH_PAGES  0x1F
#  define DEVICE_INFO_BL_TYPE      0x06
#endif

uint8_t deviceInfo[9] = {0x34, 0x37, 0x31, BOOTLOADER_VERSION_NUM,
                         DEVICE_INFO_FLASH_PAGES,
                         DEVICE_INFO_BL_TYPE,
                         0x06, 0x01, 0x30};

uint8_t rxBuffer[258];
uint8_t payLoadBuffer[256];
char rxbyte=0;
uint32_t address;


typedef union __attribute__ ((packed))
{
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;
uint16_t len;
uint8_t received_crc_low_byte;
uint8_t received_crc_high_byte;
uint8_t calculated_crc_low_byte;
uint8_t calculated_crc_high_byte;
uint16_t payload_buffer_size;
char incoming_payload_no_command = 0;




static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
static void MX_GPIO_INPUT_INIT(void);
//void processmessage(void);
void serialwriteChar(char data);
void sendString(uint8_t data[], int len);
void recieveBuffer();

#define BAUDRATE          (19200)
#define BITTIME           (1000000/BAUDRATE)
#define HALFBITTIME       (500000/BAUDRATE)




void delayMicroseconds(uint32_t micros)
{
    SysTick->CNT = 0;
    while (SysTick->CNT < micros*TIME_FACTOR)
    {

    }
}

void jumpToMainApp(void)
{
    //__disable_irq();
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
    for (volatile uint32_t i = 0; i < 1440000; i++) { __asm__("nop"); }
    NVIC_EnableIRQ(Software_IRQn);
    NVIC_SetPendingIRQ(Software_IRQn);
}

void makeCrc(uint8_t* pBuff, uint16_t length)
{
    static uint8_16_u CRC_16;
    CRC_16.word = 0;

    for (int i = 0; i < length; i++)
    {
        uint8_t xb = pBuff[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) != 0)
            {
                CRC_16.word = CRC_16.word >> 1;
                CRC_16.word = CRC_16.word ^ 0xA001;
            }
            else
            {
                CRC_16.word = CRC_16.word >> 1;
            }
            xb = xb >> 1;
        }
    }
    calculated_crc_low_byte  = CRC_16.bytes[0];
    calculated_crc_high_byte = CRC_16.bytes[1];
}

char checkCrc(uint8_t* pBuff, uint16_t length)
{
    unsigned char received_crc_low_byte2  = pBuff[length];
    unsigned char received_crc_high_byte2 = pBuff[length + 1];
    makeCrc(pBuff, length);

    if ((calculated_crc_low_byte  == (uint8_t)received_crc_low_byte2) &&
        (calculated_crc_high_byte == (uint8_t)received_crc_high_byte2))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


void setReceive()
{
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x8<<0);
}

void setTransmit()
{
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x3<<0);
}



void send_ACK()
{
    setTransmit();
    serialwriteChar(0x30); // good ack
    setReceive();
}

void send_BAD_ACK()
{
    setTransmit();
    serialwriteChar(0xC1); // bad command
    setReceive();
}

void send_BAD_CRC_ACK()
{
    setTransmit();
    serialwriteChar(0xC2); // bad CRC
    setReceive();
}

void sendDeviceInfo()
{
    setTransmit();
    sendString(deviceInfo, 9);
    setReceive();
}

bool checkAddressWritable(uint32_t address)
{
    return address >= APPLICATION_ADDRESS;
}

void decodeInput()
{
    if (incoming_payload_no_command)
    {
        len = payload_buffer_size;
        if (checkCrc(rxBuffer, len))
        {
            memset(payLoadBuffer, 0, sizeof(payLoadBuffer));
            for (int i = 0; i < len; i++)
            {
                payLoadBuffer[i] = rxBuffer[i];
            }
            send_ACK();
            incoming_payload_no_command = 0;
            return;
        }
        else
        {
            send_BAD_CRC_ACK();
            return;
        }
    }

    cmd = rxBuffer[0];

    if (rxBuffer[16] == 0x7d)
    {
        if (rxBuffer[8] == 13 && rxBuffer[9] == 66)
        {
            sendDeviceInfo();
            rxBuffer[20] = 0;
        }
        return;
    }

    if (rxBuffer[20] == 0x7d)
    {
        if (rxBuffer[12] == 13 && rxBuffer[13] == 66)
        {
            sendDeviceInfo();
            rxBuffer[20] = 0;
            return;
        }
    }

    if (rxBuffer[40] == 0x7d)
    {
        if (rxBuffer[32] == 13 && rxBuffer[33] == 66)
        {
            sendDeviceInfo();
            rxBuffer[20] = 0;
            return;
        }
    }

    if (cmd == CMD_RUN)
    {
        if ((rxBuffer[1] == 0) && (rxBuffer[2] == 0) && (rxBuffer[3] == 0))
        {
            invalid_command = 101;
        }
    }

    if (cmd == CMD_PROG_FLASH)
    {
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
            send_BAD_CRC_ACK();
            return;
        }
        if (!checkAddressWritable(address))
        {
            send_BAD_ACK();
            return;
        }
        save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size, address);
        send_ACK();
        return;
    }

    if (cmd == CMD_SET_ADDRESS)
    {
        len = 4;
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
            send_BAD_CRC_ACK();
            return;
        }
        invalid_command = 0;
        address = CH32_FLASH_START + (rxBuffer[2] << 8 | rxBuffer[3]);
        send_ACK();
        return;
    }

    if (cmd == CMD_SET_BUFFER)
    {
        len = 4;
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
            send_BAD_CRC_ACK();
            return;
        }
        if (rxBuffer[2] == 0x01)
        {
            payload_buffer_size = 256;
        }
        else
        {
            payload_buffer_size = rxBuffer[3];
        }
        incoming_payload_no_command = 1;
        address_expected_increment = 256;
        setReceive();
        return;
    }

    if (cmd == CMD_KEEP_ALIVE)
    {
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
            send_BAD_CRC_ACK();
            return;
        }
        setTransmit();
        serialwriteChar(0xC1);
        setReceive();
        return;
    }

    if (cmd == CMD_ERASE_FLASH)
    {
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
            send_BAD_CRC_ACK();
            return;
        }
        if (!checkAddressWritable(address))
        {
            send_BAD_ACK();
            return;
        }
        send_ACK();
        return;
    }

    if (cmd == CMD_READ_EEPROM)
    {
        eeprom_req = 1;
    }

    if (cmd == CMD_READ_FLASH_SIL)
    {
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
            send_BAD_CRC_ACK();
            return;
        }
        count++;
        uint16_t out_buffer_size = rxBuffer[1];
        if (out_buffer_size == 0)
        {
            out_buffer_size = 256;
        }
        address_expected_increment = 128;

        setTransmit();
        uint8_t read_data[out_buffer_size + 3];
        memset(read_data, 0, sizeof(read_data));
        read_flash_bin((uint8_t*)read_data, address, out_buffer_size);
        makeCrc(read_data, out_buffer_size);
        read_data[out_buffer_size]     = calculated_crc_low_byte;
        read_data[out_buffer_size + 1] = calculated_crc_high_byte;
        read_data[out_buffer_size + 2] = 0x30;
        sendString(read_data, out_buffer_size + 3);
        setReceive();
        return;
    }

    setTransmit();
    serialwriteChar(0xC1); // unknown command
    invalid_command++;
    setReceive();
}


void serialreadChar(void)
{
    rxbyte = 0;
    while (!(input_port->INDR & input_pin))
    {
        if (SysTick->CNT > (200000 * TIME_FACTOR))
        {
            invalid_command = 101;
            return;
        }
    }
    while (input_port->INDR & input_pin)
    {
        if (SysTick->CNT > (250 * TIME_FACTOR) && messagereceived)
        {
            return;
        }
    }
    delayMicroseconds(HALFBITTIME);

    int bits_to_read = 0;
    while (bits_to_read < 8)
    {
        delayMicroseconds(BITTIME);
        rxbyte = rxbyte | (((input_port->INDR & input_pin)) >> PIN_NUMBER) << bits_to_read;
        bits_to_read++;
    }
    delayMicroseconds(HALFBITTIME);
    messagereceived = 1;
    receviedByte = rxbyte;
}


void serialwriteChar(char data)
{
    input_port->BCR = input_pin; // start bit
    char bits_to_read = 0;
    while (bits_to_read < 8)
    {
        delayMicroseconds(BITTIME);
        if (data & 0x01)
        {
            input_port->BSHR = input_pin;
        }
        else
        {
            input_port->BCR = input_pin;
        }
        bits_to_read++;
        data = data >> 1;
    }
    delayMicroseconds(BITTIME);
    input_port->BSHR = input_pin; // stop bit
}

void sendString(uint8_t *data, int len)
{
    for (int i = 0; i < len; i++)
    {
        serialwriteChar(data[i]);
        delayMicroseconds(BITTIME);
    }
}


void recieveBuffer(void)
{
    count = 0;
    messagereceived = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    for (int i = 0; i < sizeof(rxBuffer); i++)
    {
        serialreadChar();
        if (incoming_payload_no_command)
        {
            if (count == payload_buffer_size + 2)
            {
                break;
            }
            rxBuffer[i] = rxbyte;
            count++;
        }
        else
        {
            if (SysTick->CNT > 250 * TIME_FACTOR)
            {
                count = 0;
                break;
            }
            else
            {
                rxBuffer[i] = rxbyte;
                if (i == 257)
                {
                    invalid_command += 20;
                }
            }
        }
    }

    decodeInput();
}


void update_EEPROM(void)
{
    rxBuffer[1] = 2;
    if (BOOTLOADER_VERSION_NUM != rxBuffer[2])
    {
        if (rxBuffer[2] == 0xFF || rxBuffer[2] == 0x00)
        {
            return;
        }
        rxBuffer[2] = BOOTLOADER_VERSION_NUM;
        rxBuffer[1] = 2;
        save_flash_nolib(rxBuffer, 48, EEPROM_START_ADD);
    }
}


void checkForSignal(void)
{
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x8<<0);
    input_port->BCR = input_pin;
    delayMicroseconds(500);

    for (int i = 0; i < 500; i++)
    {
        if (!(input_port->INDR & input_pin))
        {
            low_pin_count++;
        }
        delayMicroseconds(10);
    }
    if (low_pin_count == 0)
    {
        return; // all high — bootloader signal present
    }
    low_pin_count = 0;

    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x4<<0);
    input_port->BCR = input_pin;
    delayMicroseconds(500);

    for (int i = 0; i < 500; i++)
    {
        if (!(input_port->INDR & input_pin))
        {
            low_pin_count++;
        }
        delayMicroseconds(10);
    }
    if (low_pin_count == 0)
    {
        return; // floating — no signal
    }
    if (low_pin_count > 0)
    {
        //jumpToMainApp();
    }
}



/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

int main(void)
{
    MX_TIM4_Init();
    MX_GPIO_INPUT_INIT();
    checkForSignal();

    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x8<<0);

    update_EEPROM();

    while (1)
    {
        recieveBuffer();
        if (invalid_command > 100)
        {
            jumpToMainApp();
        }
    }
}


static void MX_GPIO_INPUT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = input_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(input_port, &GPIO_InitStructure);
}

static void MX_TIM4_Init(void)
{
    SysTick->CTLR=0;
    SysTick->SR = 0;
    SysTick->CNT=0;
    SysTick->CMP = (uint64_t)(-1);
    SysTick->CTLR = 0x9;
}

