//
// Created by bohm on 1/31/24.
//

#include <stdio.h>
#include <freertos/portmacro.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#ifndef BNO08X_TEST_BNO08X_H
#define BNO08X_TEST_BNO08X_H

#include <esp_err.h>

#define BNO08X_SENSOR_ADDR                 0x4A

// Page 0
#define BNO08X_ID (0x00)           // was (0xA0)
#define BNO08X_CHIP_ID 0x00        // value: 0xA0
#define BNO08X_SW_REV_ID_LSB 0x04  // value: 0x08
#define BNO08X_SW_REV_ID_MSB 0x05  // value: 0x03
#define BNO08X_BL_REV_ID 0x06      // N/A
#define BNO08X_PAGE_ID 0x07

#define BNO08X_SYS_TRIGGER 0x3F

#define BNO08X_TEMP 0x34
#define BNO08X_CALIB_STAT 0x35
#define BNO08X_ST_RESULT 0x36
#define BNO08X_INT_STATUS 0x37
#define BNO08X_SYS_CLK_STATUS 0x38
#define BNO08X_SYS_STATUS 0x39
#define BNO08X_SYS_ERR 0x3A
#define BNO08X_UNIT_SEL 0x3B
#define BNO08X_OPR_MODE 0x3D
#define BNO08X_PWR_MODE 0x3E
#define BNO08X_SYS_TRIGGER 0x3F
#define BNO08X_TEMP_SOURCE 0x40
#define BNO08X_AXIS_MAP_CONFIG 0x41
#define BNO08X_AXIS_MAP_SIGN 0x42
#define BNO08X_ACC_OFFSET_X_LSB 0x55

typedef enum {  // BNO-08x operation modes
  BNO08X_OPERATION_MODE_CONFIG = 0x00,
  // Sensor Mode
  BNO08X_OPERATION_MODE_ACCONLY,
  BNO08X_OPERATION_MODE_MAGONLY,
  BNO08X_OPERATION_MODE_GYRONLY,
  BNO08X_OPERATION_MODE_ACCMAG,
  BNO08X_OPERATION_MODE_ACCGYRO,
  BNO08X_OPERATION_MODE_MAGGYRO,
  BNO08X_OPERATION_MODE_AMG,  // 0x07
                              // Fusion Mode
  BNO08X_OPERATION_MODE_IMU,
  BNO08X_OPERATION_MODE_COMPASS,
  BNO08X_OPERATION_MODE_M4G,
  BNO08X_OPERATION_MODE_NDOF_FMC_OFF,
  BNO08X_OPERATION_MODE_NDOF  // 0x0C
} bno08x_opmode_t;

typedef enum {
  BNO08X_VECTOR_ACCELEROMETER = 0x08,  // Default: m/s²
  BNO08X_VECTOR_MAGNETOMETER = 0x0E,   // Default: uT
  BNO08X_VECTOR_GYROSCOPE = 0x14,      // Default: rad/s
  BNO08X_VECTOR_EULER = 0x1A,          // Default: degrees
  BNO08X_VECTOR_QUATERNION = 0x20,     // No units
  BNO08X_VECTOR_LINEARACCEL = 0x28,    // Default: m/s²
  BNO08X_VECTOR_GRAVITY = 0x2E         // Default: m/s²
} bno08x_vector_type_t;

esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data);

void bno08x_setPage(uint8_t page);
void bno08x_delay(int time);
void bno08x_setOperationMode(bno08x_opmode_t mode);
void bno08x_setOperationModeConfig();
void bno08x_setOperationModeNDOF();
void bno08x_task(void *pvParams);

#endif //BNO08X_TEST_BNO08X_H
