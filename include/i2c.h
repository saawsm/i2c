#ifndef _I2C_H
#define _I2C_H

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_DEV_MUTEX_TAKE(dev)                                                                                                                                \
   do {                                                                                                                                                        \
      esp_err_t __ = i2c_dev_mutex_take((dev));                                                                                                                \
      if (__ != ESP_OK)                                                                                                                                        \
         return __;                                                                                                                                            \
   } while (0)

#define I2C_DEV_MUTEX_GIVE(dev)                                                                                                                                \
   do {                                                                                                                                                        \
      esp_err_t __ = i2c_dev_mutex_give((dev));                                                                                                                \
      if (__ != ESP_OK)                                                                                                                                        \
         return __;                                                                                                                                            \
   } while (0)

/**
 * Represents an I2C device on a I2C bus
 */
typedef struct {
   i2c_port_t port;         // I2C port number
   uint8_t address;         // Device address
   SemaphoreHandle_t mutex; // Device mutex, for devices that depend on multiple sequential writes or reads
} i2c_dev_t;

/**
 * @brief Init library
 *
 * This function must be called before calling any other functions from this library.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_init();

/**
 * @brief Cleanup library
 *
 * Uninstall I2C drivers. Must call i2c_init() again, before being able to call any other functions.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_free();

/**
 * @brief Initialize I2C port for use
 *
 * Initializes and installs the I2C driver as master for the specified I2C port.
 * Function is thread safe
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_port_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed);

/**
 * @brief Cleanup I2C port
 *
 * Deinitializes and uninstalls the I2C driver for the specified I2C port.
 * Function is thread safe
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_port_free(i2c_port_t port);

/**
 * @brief Setup the provided I2C device descriptor
 *
 * Function is thread safe
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_desc(i2c_dev_t* dev, i2c_port_t port, uint8_t address);

/**
 * @brief Create mutex for device
 *
 * Create a device mutex stored in \p dev for transactional device multi-write/reads.
 *
 * @param dev Device descriptor
 * @return ESP_OK on sucess
 */
esp_err_t i2c_dev_mutex_create(i2c_dev_t* dev);

/**
 * @brief Deletes mutex for device
 *
 * Deletes the stored device mutex created by i2c_dev_mutex_create().
 *
 * @param dev Device descriptor
 * @return ESP_OK on sucess
 */
esp_err_t i2c_dev_mutex_delete(i2c_dev_t* dev);

/**
 * @brief Unlocks the device
 *
 * Releases the mutex back to the device.
 *
 * @param dev Device descriptor
 * @return ESP_OK on sucess
 */
esp_err_t i2c_dev_mutex_give(i2c_dev_t* dev);

/**
 * @brief Locks the device
 *
 * Attempts to take the mutex from the device (aka locks). If timeout occurs ESP_ERR_TIMEOUT is returned.
 *
 * @param dev Device descriptor
 * @return ESP_OK on sucess, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t i2c_dev_mutex_take(i2c_dev_t* dev);

/**
 * @brief Write to slave device
 *
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 *
 * Function is thread safe
 *
 * @param dev Device descriptor
 * @param out_reg Pointer to register address to send if not null
 * @param out_reg_size Size of register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write(const i2c_dev_t* dev, const void* out_reg, size_t out_reg_size, const void* out_data, size_t out_size);

/**
 * @brief Write to register with an 8-bit address
 *
 * Function is thread safe
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write_reg(const i2c_dev_t* dev, uint8_t reg, const void* out_data, size_t out_size);

/**
 * @brief Read from slave device
 *
 * Function is thread safe
 *
 * Issue a send operation of \p out_data register address, followed by reading \p in_size bytes
 * from slave into \p in_data.
 *
 * @param dev Device descriptor
 * @param out_data Pointer to data to send if not null
 * @param out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read(const i2c_dev_t* dev, const void* out_data, size_t out_size, void* in_data, size_t in_size);

/**
 * @brief Read from register with an 8-bit address
 *
 * Function is thread safe
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read_reg(const i2c_dev_t* dev, uint8_t reg, void* in_data, size_t in_size);

#ifdef __cplusplus
}
#endif

#endif