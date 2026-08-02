#ifndef TEST1_LCD_PINS_H
#define TEST1_LCD_PINS_H

#include "stm32h7xx_hal.h"

/* ST7789 SPI TFT (240x280) wiring on DM-MC02 / STM32H723VGT6.
 *
 * SPI1 bus:  SCK = PB3 (AF5), MOSI = PD7 (AF5), MISO unused (TX only).
 * Control:   CS = PE15, BLK = PB10, RES = PB11, DC = PD10.
 *
 * SPI2 stays reserved for the BMI088 IMU and must not be touched here.
 */

#define LCD_CS_Pin          GPIO_PIN_15
#define LCD_CS_GPIO_Port    GPIOE

#define LCD_BLK_Pin         GPIO_PIN_10
#define LCD_BLK_GPIO_Port   GPIOB

#define LCD_RES_Pin         GPIO_PIN_11
#define LCD_RES_GPIO_Port   GPIOB

#define LCD_DC_Pin          GPIO_PIN_10
#define LCD_DC_GPIO_Port    GPIOD

extern SPI_HandleTypeDef hspi1;

#endif
