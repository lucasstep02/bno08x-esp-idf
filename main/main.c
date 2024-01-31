
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "bno055.h"

static const char *TAG = "bno055-test";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000



static esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BNO055_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BNO055_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

void bno055_setPage(uint8_t page) {
    register_write_byte(BNO055_PAGE_ID, page);
}

void bno055_delay(int time) {
    vTaskDelay(time / portTICK_RATE_MS);
}

void bno055_setOperationMode(bno055_opmode_t mode) {
    register_write_byte(BNO055_OPR_MODE, mode);
    if (mode == BNO055_OPERATION_MODE_CONFIG) {
        bno055_delay(19);
    } else {
        bno055_delay(7);
    }
}

void bno055_setOperationModeConfig() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void bno055_task(void *pvParams) {
    // reset
    ESP_LOGI(TAG, "Going to reset the BNO");
    register_write_byte(0x00, 0x00);
    bno055_delay(700);

    uint8_t id = 0;
    register_read(BNO055_CHIP_ID, &id, 1);
    if (id != BNO055_ID) {
        ESP_LOGE(TAG, "Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
    } else {
        ESP_LOGI(TAG, "BNO055 Found and reset");

        bno055_setPage(0);
        register_write_byte(BNO055_SYS_TRIGGER, 0x0);

        // Select BNO055 config mode
        bno055_setOperationModeConfig();
        bno055_delay(10);

        bno055_setOperationModeNDOF();

        uint8_t frequency = 50;
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = frequency / portTICK_PERIOD_MS;

        int16_t bno_raw_euler[8];

        while (1) {
            xLastWakeTime = xTaskGetTickCount();
            register_read(BNO055_VECTOR_EULER, bno_raw_euler, 6);
            ESP_LOGI(TAG, "Yaw: %d, Roll: %d, Pitch: %d", bno_raw_euler[0], bno_raw_euler[1], bno_raw_euler[2]);

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
    vTaskDelete(NULL);
}

void blink_led(void *pvParameters);

void app_main(void) {
    xTaskCreate( blink_led, "LED Task", 4096, NULL, 150, NULL);

    ESP_LOGI(TAG, "Going to init the I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    xTaskCreate( bno055_task, "BNO055 Task", 4096, NULL, 10, NULL);

}
