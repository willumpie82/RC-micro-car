#ifndef RCBOARDH
#define RCBOARDH

//#define CONFIG_HAS_TONE
#define CONFIG_HAS_NRF24
//#define CONFIG_HAS_SBUS
//#define CONFIG_HAS_LIGHTS
//#define CONFIG_HAS_SERVO


#define PIN_ADC_VBAT    PA0
//NRF24 pins
#define PIN_NRF_CE      PA1
#define PIN_NRF_CSN     PA3
#define PIN_NRF_SCK     PA5
#define PIN_NRF_MISO    PA6
#define PIN_NRF_MOSI    PA7
#define PIN_NRF_IRQ     PB1

//MPU6050 pins
#define PIN_MPU_INT     PB0
#define PIN_MPU_SDA     PB7
#define PIN_MPU_SCL     PB6

//MC34933 motordrive pins
#define PIN_MOTOR_M2A   PA15
#define PIN_MOTOR_M2B   PA12
#define PIN_MOTOR_M1A   PA10
#define PIN_MOTOR_M1B   PA11

//CPU pins
#define PIN_MCU_BOOT1   PB2
#define PIN_MCU_SWCLK   PA14
#define PIN_MCU_SWDIO   PA13
#define PIN_MCU_TX      PA9     //testpoint (label swapped on board v1)
#define PIN_MCU_RX      PA8     //testpoint (label swapped on board v1)

//spare pins
#define PIN_SPARE1      PA4     //testpoint
#define PIN_SPARE2      PA2     //testpoint

//board settings
#define SERIAL_BAUDRATE    115200
#define SERIAL_USART1_BRR  0x271
#define CLOCK_SPEED_400KHZ 400000

#endif //RCBOARDH