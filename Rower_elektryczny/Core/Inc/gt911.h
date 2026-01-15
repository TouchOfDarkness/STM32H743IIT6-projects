/*
 * gt911.h
 *
 *  Created on: Jan 15, 2026
 *      Author: romel
 */

#ifndef INC_GT911_H_
#define INC_GT911_H_
#include "main.h"
#include "i2c.h"

// Adresy I2C GT911
#define GT911_ADDR1         0xBA  // (0x5D << 1)
#define GT911_ADDR2         0x28  // (0x14 << 1)

// Rejestry GT911
#define GT911_PRODUCT_ID    0x8140
#define GT911_CONFIG_REG    0x8047
#define GT911_COMMAND_REG   0x8040
#define GT911_READ_STATUS   0x814E
#define GT911_POINT_INFO    0x814F

// Maksymalna liczba punktów dotyku
#define GT911_MAX_TOUCHES   5

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t size;
    uint8_t trackId;
} GT911_TouchPoint;

typedef struct {
    uint8_t touchCount;
    GT911_TouchPoint points[GT911_MAX_TOUCHES];
} GT911_TouchData;

// Funkcje publiczne
uint8_t GT911_Init(I2C_HandleTypeDef *hi2c);
uint8_t GT911_ReadTouchData(I2C_HandleTypeDef *hi2c, GT911_TouchData *touchData);
void GT911_Reset(void);


#endif /* INC_GT911_H_ */
