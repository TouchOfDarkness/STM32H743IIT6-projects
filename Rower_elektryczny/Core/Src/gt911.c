#include "gt911.h"
#include <string.h>

static uint8_t gt911_address = GT911_ADDR1;

// Funkcja pomocnicza - zapis do rejestru
static HAL_StatusTypeDef GT911_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *data, uint16_t len) {
    uint8_t buffer[256];
    buffer[0] = (reg >> 8) & 0xFF;
    buffer[1] = reg & 0xFF;
    memcpy(&buffer[2], data, len);
    return HAL_I2C_Master_Transmit(hi2c, gt911_address, buffer, len + 2, 100);
}

// Funkcja pomocnicza - odczyt z rejestru
static HAL_StatusTypeDef GT911_ReadReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *data, uint16_t len) {
    uint8_t regAddr[2];
    regAddr[0] = (reg >> 8) & 0xFF;
    regAddr[1] = reg & 0xFF;

    if (HAL_I2C_Master_Transmit(hi2c, gt911_address, regAddr, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(hi2c, gt911_address, data, len, 100);
}

// Reset GT911 - WAŻNE: ustaw adres I2C
void GT911_Reset(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Skonfiguruj INT jako WYJŚCIE, aby wymusić stan logiczny
    GPIO_InitStruct.Pin = GT911_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GT911_INT_GPIO_Port, &GPIO_InitStruct);

    // 2. Sekwencja Resetu (Ustawienie adresu 0xBA -> INT LOW)
    HAL_GPIO_WritePin(GT911_INT_GPIO_Port, GT911_INT_Pin, GPIO_PIN_RESET); // INT LOW
    HAL_GPIO_WritePin(GT911_RST_GPIO_Port, GT911_RST_Pin, GPIO_PIN_RESET); // RST LOW
    HAL_Delay(10);

    HAL_GPIO_WritePin(GT911_RST_GPIO_Port, GT911_RST_Pin, GPIO_PIN_SET);   // RST HIGH
    HAL_Delay(10);

    // 3. Przywróć INT jako WEJŚCIE (Przerwanie)
    // Czekamy chwilę, aż GT911 "złapie" adres
    HAL_Delay(50);

    GPIO_InitStruct.Pin = GT911_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GT911_INT_GPIO_Port, &GPIO_InitStruct);

    HAL_Delay(50);
}

// Inicjalizacja GT911
uint8_t GT911_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t productId[4];

    // Reset kontrolera
    GT911_Reset();

    // Sprawdź adres I2C
    if (HAL_I2C_IsDeviceReady(hi2c, GT911_ADDR1, 3, 100) == HAL_OK) {
        gt911_address = GT911_ADDR1;
    } else if (HAL_I2C_IsDeviceReady(hi2c, GT911_ADDR2, 3, 100) == HAL_OK) {
        gt911_address = GT911_ADDR2;
    } else {
        return 0; // Błąd - nie znaleziono urządzenia
    }

    // Odczytaj Product ID
    if (GT911_ReadReg(hi2c, GT911_PRODUCT_ID, productId, 4) != HAL_OK) {
        return 0;
    }

    // Sprawdź czy to GT911 (Product ID: "911")
    if (productId[0] != '9' || productId[1] != '1' || productId[2] != '1') {
        return 0;
    }

    return 1; // Sukces
}

// Odczyt danych dotykowych
uint8_t GT911_ReadTouchData(I2C_HandleTypeDef *hi2c, GT911_TouchData *touchData) {
    uint8_t status;
    uint8_t touchBuffer[40];

    // Odczytaj status
    if (GT911_ReadReg(hi2c, GT911_READ_STATUS, &status, 1) != HAL_OK) {
        return 0;
    }

    // Sprawdź czy są nowe dane
    if ((status & 0x80) == 0) {
        return 0; // Brak nowych danych
    }

    // Liczba punktów dotyku
    touchData->touchCount = status & 0x0F;

    if (touchData->touchCount > 0 && touchData->touchCount <= GT911_MAX_TOUCHES) {
        // Odczytaj dane punktów
        if (GT911_ReadReg(hi2c, GT911_POINT_INFO, touchBuffer, touchData->touchCount * 8) != HAL_OK) {
            return 0;
        }

        // Parsuj dane
        for (uint8_t i = 0; i < touchData->touchCount; i++) {
            uint8_t *p = &touchBuffer[i * 8];
            touchData->points[i].trackId = p[0];
            touchData->points[i].x = (uint16_t)(p[1] | (p[2] << 8));
            touchData->points[i].y = (uint16_t)(p[3] | (p[4] << 8));
            touchData->points[i].size = (uint16_t)(p[5] | (p[6] << 8));
        }
    }

    // Wyczyść flagę gotowości
    status = 0;
    GT911_WriteReg(hi2c, GT911_READ_STATUS, &status, 1);

    return 1;
}
