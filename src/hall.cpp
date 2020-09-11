#include "Arduino.h"
#include "hall.h"
#include "rc_board.h"
#include "Wire.h"


void init_cpu( void )
{
  //setup system clock (12Mhz xtal)
  RCC->CR |= (RCC_CR_PLLON_Msk & !RCC_CR_PLLON);        //Turn PLL off
  RCC->CR |= RCC_HSE_ON;
  while (!(RCC->CR & RCC_CR_HSERDY));
  RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;  
  RCC->CFGR |= (RCC_CFGR_PLLMULL_Msk & RCC_PLL_MUL6);  //Set PLL multiplier
  RCC->CFGR &= ~RCC_CFGR_PLLSRC; //set PLL to HSE
// Turn On PLL Clock
  RCC->CR |= RCC_CR_PLLON;
  // Wait Until PLL Is Ready
  while (!(RCC->CR & RCC_CR_PLLRDY));
  // Set System To PLL CLock
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  // Clear All Interrupts
  RCC->CIR = 0x009F0000;

}

void init_gpio( void )
{
  // initialize LED digital pin as an output.
  pinMode(PIN_SPARE2, OUTPUT);
}

void init_serial( void )
{
  // initialize serial communication
  Serial.begin(SERIAL_BAUDRATE);
  USART1->BRR = 0x271;
  Serial.printf("init serial to %i baud",SERIAL_BAUDRATE);
}

void init_i2c( void )
{
  Wire.begin();
  Wire.setClock(CLOCK_SPEED_400KHZ);
}


void hall_init()
{
  init_cpu();
  init_gpio();
  init_serial();
  init_i2c();
}
