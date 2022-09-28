#ifndef ESP32S3_RGB_MATRIX_H
#define ESP32S3_RGB_MATRIX_H
#include <stdint.h>
#include <driver/gpio.h>

//A, B, C, D must be on.config.pins.<40
#define PIN_R1 GPIO_NUM_18
#define PIN_G1 GPIO_NUM_8
#define PIN_B1 GPIO_NUM_17
#define PIN_R2 GPIO_NUM_16
#define PIN_G2 GPIO_NUM_19
#define PIN_B2 GPIO_NUM_15
#define EXTRA_SPI_DATAPIN_1 GPIO_NUM_3
#define EXTRA_SPI_DATAPIN_2 GPIO_NUM_45
#define NUM_DATA_PINS 6
#define PIN_A GPIO_NUM_7
#define PIN_B GPIO_NUM_9
#define PIN_C GPIO_NUM_6
#define PIN_D GPIO_NUM_10
#define PIN_E GPIO_NUM_20
#define NUM_ROW_PINS 5
#define PIN_OE GPIO_NUM_4
#define PIN_LAT GPIO_NUM_11
#define PIN_CLK GPIO_NUM_5

#define PANEL_FRAMERATE (30)
#define PANEL_NUM_ELECTRICAL_ROWS (16)
#define PANEL_ELECTRICAL_ROW_LENGTH (1024) //top half of panel only, as we have two sets of rgb pins
#define PLANE_SIZE (PANEL_NUM_ELECTRICAL_ROWS*PANEL_ELECTRICAL_ROW_LENGTH)
#define BUFFER_NUM_PLANES (4)
#define BUFFER_SIZE (BUFFER_NUM_PLANES*PLANE_SIZE)

#define PANEL_PX_WIDTH (256)
#define PANEL_PX_HEIGHT (128)

#define SPI_CLOCK_HZ (20 * 1000000) 
//20MHz lower this if some pixels fail to light up or it looks like its having stability issues. 20MHz should definitely work for two panels
//I started experiencing errors as 34MHz, so i've allowed ~10% margin by setting it to 30MHz
//playing with the clock vs data line timing may also help get you higher clock speeds
void initRgbPanel();
void swapBuffers(bool copyBuffer);
void clearBuffer();
void drawPixel(int row, int col, uint16_t color);
void drawPixelElectricalPos(int row, int col, uint16_t color);
#endif
