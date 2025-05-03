#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C configuration
#define I2C_MASTER_SCL_IO           22          /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */

// BME280 I2C address
#define BME280_ADDR                 0x76

// BME280 registers
#define REG_RESET                   0xE0
#define REG_ID                      0xD0
#define REG_CTRL_HUM                0xF2
#define REG_STATUS                  0xF3
#define REG_CTRL_MEAS               0xF4
#define REG_CONFIG                  0xF5
#define REG_PRESS_MSB               0xF7
#define REG_PRESS_LSB               0xF8
#define REG_PRESS_XLSB              0xF9
#define REG_TEMP_MSB                0xFA
#define REG_TEMP_LSB                0xFB
#define REG_TEMP_XLSB               0xFC
#define REG_HUM_MSB                 0xFD
#define REG_HUM_LSB                 0xFE

// Calibration parameters
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1;
static int16_t  dig_H2;
static uint8_t  dig_H3;
static int16_t  dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Write single byte to a reg
static esp_err_t bme280_write_reg(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Read multiple bytes
static esp_err_t bme280_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t bme280_read_calibration(void) {
    uint8_t calib[26];
    esp_err_t err = bme280_read_regs(0x88, calib, 26);
    if (err != ESP_OK) return err;
    dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
    dig_H1 = calib[25];

    uint8_t hum_cal[7];
    err = bme280_read_regs(0xE1, hum_cal, 7);
    if (err != ESP_OK) return err;
    dig_H2 = (int16_t)(hum_cal[1] << 8 | hum_cal[0]);
    dig_H3 = hum_cal[2];
    dig_H4 = (int16_t)((hum_cal[3] << 4) | (hum_cal[4] & 0x0F));
    dig_H5 = (int16_t)((hum_cal[5] << 4) | (hum_cal[4] >> 4));
    dig_H6 = (int8_t)hum_cal[6];
    return ESP_OK;
}

static float bme280_compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)dig_T1) * ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
    t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8;
    return T / 100.0;
}

static float bme280_compensate_P(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)dig_P1)) >> 33;
    if (var1 == 0) return 0; // avoid exception
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return p / 256.0;
}

static float bme280_compensate_H(int32_t adc_H) {
    int32_t v_x1 = t_fine - 76800;
    v_x1 = (((((adc_H << 14) - ((int32_t)dig_H4 << 20) -((int32_t)dig_H5 * v_x1)) + 16384) >> 15)
            * (((((((v_x1 * (int32_t)dig_H6) >> 10) * (((v_x1 * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152)
            * (int32_t)dig_H2 + 8192) >> 14));
    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
    v_x1 = (v_x1 < 0) ? 0 : v_x1;
    v_x1 = (v_x1 > 419430400) ? 419430400 : v_x1;
    float h = (v_x1 >> 12);
    return h / 1024.0;
}

static esp_err_t bme280_init(void) {
    // reset
    bme280_write_reg(REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(300));
    // read calibration
    esp_err_t err = bme280_read_calibration();
    if (err != ESP_OK) return err;
    // ctrl_hum oversampling x1
    bme280_write_reg(REG_CTRL_HUM, 0x01);
    // ctrl_meas: temp x1, press x1, mode normal
    bme280_write_reg(REG_CTRL_MEAS, 0x27);
    // config: standby 1s, filter off
    bme280_write_reg(REG_CONFIG, 0xA0);
    return ESP_OK;
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI("BME280", "I2C initialized");
    ESP_ERROR_CHECK(bme280_init());
    ESP_LOGI("BME280", "Sensor initialized");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint8_t data[8];
        ESP_ERROR_CHECK(bme280_read_regs(REG_PRESS_MSB, data, 8));
        int32_t raw_p = (int32_t)((uint32_t)data[0] << 12 | (uint32_t)data[1] << 4 | data[2] >> 4);
        int32_t raw_t = (int32_t)((uint32_t)data[3] << 12 | (uint32_t)data[4] << 4 | data[5] >> 4);
        int32_t raw_h = (int32_t)((uint32_t)data[6] << 8  | data[7]);

        float temp = bme280_compensate_T(raw_t);
        float press = bme280_compensate_P(raw_p) / 100.0;
        float hum  = bme280_compensate_H(raw_h);

        ESP_LOGI("BME280", "Temp: %.2f C, Pressure: %.2f hPa, Humidity: %.2f %%", temp, press, hum);
    }
}
