#ifndef ESP32S3_RGB_MATRIX_H
#define ESP32S3_RGB_MATRIX_H
#include <stdint.h>
void initRgbPanel();
void swapBuffers(bool copyBuffer);
void clearBuffer();
inline void drawPixel(int row, int col, uint16_t color);
#endif