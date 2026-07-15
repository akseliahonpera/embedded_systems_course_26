#ifndef BMP581_PORT_H
#define BMP581_PORT_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "bmp5.h"

/**
 * @brief This function registers the BMP581 on an initialized I2C bus
 * 
 * @param bus Handle of the initialized I2C master bus
 * @param i2c_freq_hz I2C frequency (Hertz)
 * @return esp_err_t 
 *     - ESP_OK: BMP581 was successfully initialized and registered.
 *     - Other esp_err_t values: Initialization failed.
 */
esp_err_t bmp581_port_init(i2c_master_bus_handle_t bus,
                           uint32_t i2c_freq_hz);

/**
 * @brief Reads data from a BMP581 register over I2C
 * 
 * @param reg_addr Address of the BMP581 register to read from
 * @param reg_data Pointer to the buffer where the received data will be stored
 * @param len Number of bytes to read
 * @param intf_ptr Pointer to the device handle
 * @return BMP5_INTF_RET_TYPE 
 *     - 0: Read operation completed successfully.
 *     - Non-zero value: Read operation failed.
 */
BMP5_INTF_RET_TYPE bmp581_i2c_read(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr);

/**
 * @brief Writes data to a BMP581 register over I2C
 * 
 * @param reg_addr Address of the BMP581 register to write to
 * @param reg_data Pointer to the buffer containing the data to write
 * @param len Number of bytes to write
 * @param intf_ptr Pointer to the device handle
 * @return BMP5_INTF_RET_TYPE 
 */
BMP5_INTF_RET_TYPE bmp581_i2c_write(
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr);

void bmp581_delay_us(uint32_t period_us, void *intf_ptr);

esp_err_t bmp581_soft_reset(void);

#endif