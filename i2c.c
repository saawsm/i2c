#include <string.h>
#include <esp_log.h>

#include "i2c.h"

static const char* TAG = "i2c";

#define CHECK_ARG(x)                                                                                                                                           \
   do {                                                                                                                                                        \
      if (!(x))                                                                                                                                                \
         return ESP_ERR_INVALID_ARG;                                                                                                                           \
   } while (0)

typedef struct {
   SemaphoreHandle_t mutex;
   i2c_config_t config;
   bool installed;
} i2c_port_state_t;

static i2c_port_state_t ports[I2C_NUM_MAX];

#define SEMAPHORE_TAKE(i)                                                                                                                                      \
   do {                                                                                                                                                        \
      if (!xSemaphoreTake(ports[(i)].mutex, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT))) {                                                                              \
         ESP_LOGE(TAG, "Could not take port mutex %d", (i));                                                                                                   \
         return ESP_ERR_TIMEOUT;                                                                                                                               \
      }                                                                                                                                                        \
   } while (0)

#define SEMAPHORE_GIVE(i)                                                                                                                                      \
   do {                                                                                                                                                        \
      if (!xSemaphoreGive(ports[(i)].mutex)) {                                                                                                                 \
         ESP_LOGE(TAG, "Could not give port mutex %d", (i));                                                                                                   \
         return ESP_FAIL;                                                                                                                                      \
      }                                                                                                                                                        \
   } while (0)

esp_err_t i2c_init() {
   memset(ports, 0, sizeof(ports));

   for (int i = 0; i < I2C_NUM_MAX; i++) {
      ports[i].mutex = xSemaphoreCreateMutex();
      if (!ports[i].mutex) {
         ESP_LOGE(TAG, "Could not create port mutex %d", i);
         return ESP_FAIL;
      }
   }

   return ESP_OK;
}

esp_err_t i2c_free() {
   for (int i = 0; i < I2C_NUM_MAX; i++) {
      if (ports[i].installed) {
         esp_err_t err = i2c_port_free(i);
         if (err != ESP_OK)
            return err;
      }

      vSemaphoreDelete(ports[i].mutex);
      ports[i].mutex = NULL;
   }

   return ESP_OK;
}

esp_err_t i2c_port_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed) {
   SEMAPHORE_TAKE(port);

   esp_err_t err = ESP_OK;
   if (ports[port].installed) {
      err = ESP_ERR_INVALID_STATE;
      goto end;
   }

   ports[port].config.mode = I2C_MODE_MASTER;
   ports[port].config.sda_io_num = sda;
   ports[port].config.scl_io_num = scl;
   ports[port].config.master.clk_speed = clk_speed;

   err = i2c_param_config(port, &ports[port].config);
   if (err != ESP_OK)
      goto end;

   err = i2c_driver_install(port, ports[port].config.mode, 0, 0, 0);
   if (err != ESP_OK)
      goto end;

   ports[port].installed = true;

end:
   SEMAPHORE_GIVE(port);
   return err;
}

esp_err_t i2c_port_free(i2c_port_t port) {
   SEMAPHORE_TAKE(port);

   esp_err_t err = ESP_OK;
   if (!ports[port].installed) {
      err = ESP_ERR_INVALID_STATE;
      goto end;
   }

   err = i2c_driver_delete(port);
   if (err == ESP_OK)
      ports[port].installed = false;

end:
   SEMAPHORE_GIVE(port);
   return err;
}

esp_err_t i2c_dev_desc(i2c_dev_t* dev, i2c_port_t port, uint8_t address) {
   CHECK_ARG(dev);
   SEMAPHORE_TAKE(port);
   dev->port = port;
   dev->address = address;
   SEMAPHORE_GIVE(port);
   return ESP_OK;
}

esp_err_t i2c_dev_mutex_create(i2c_dev_t* dev) {
   CHECK_ARG(dev);

   ESP_LOGV(TAG, "[0x%02x on port %d] creating mutex", dev->address, dev->port);

   dev->mutex = xSemaphoreCreateMutex();
   if (!dev->mutex) {
      ESP_LOGE(TAG, "[0x%02x on port %d] Could not create device mutex", dev->address, dev->port);
      return ESP_FAIL;
   }

   return ESP_OK;
}

esp_err_t i2c_dev_mutex_delete(i2c_dev_t* dev) {
   CHECK_ARG(dev);

   ESP_LOGV(TAG, "[0x%02x on port %d] deleting mutex", dev->address, dev->port);

   vSemaphoreDelete(dev->mutex);
   return ESP_OK;
}

esp_err_t i2c_dev_mutex_take(i2c_dev_t* dev) {
   CHECK_ARG(dev);

   ESP_LOGV(TAG, "[0x%02x on port %d] taking mutex", dev->address, dev->port);

   if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT))) {
      ESP_LOGE(TAG, "[0x%02x on port %d] Could not take device mutex", dev->address, dev->port);
      return ESP_ERR_TIMEOUT;
   }
   return ESP_OK;
}

esp_err_t i2c_dev_mutex_give(i2c_dev_t* dev) {
   CHECK_ARG(dev);

   ESP_LOGV(TAG, "[0x%02x on port %d] giving mutex", dev->address, dev->port);

   if (!xSemaphoreGive(dev->mutex)) {
      ESP_LOGE(TAG, "[0x%02x on port %d] Could not give device mutex", dev->address, dev->port);
      return ESP_FAIL;
   }
   return ESP_OK;
}

esp_err_t i2c_dev_write(const i2c_dev_t* dev, const void* out_reg, size_t out_reg_size, const void* out_data, size_t out_size) {
   CHECK_ARG(dev && out_data);

   SEMAPHORE_TAKE(dev->port);

   if (!ports[dev->port].installed) {
      SEMAPHORE_GIVE(dev->port);
      return ESP_ERR_INVALID_STATE;
   }

   esp_err_t err = ESP_OK;

   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   assert(cmd != NULL);

   err = i2c_master_start(cmd);
   if (err != ESP_OK)
      goto end;

   err = i2c_master_write_byte(cmd, dev->address << 1 | I2C_MASTER_WRITE, true);
   if (err != ESP_OK)
      goto end;

   if (out_reg && out_reg_size) {
      err = i2c_master_write(cmd, (void*)out_reg, out_reg_size, true);
      if (err != ESP_OK)
         goto end;
   }

   err = i2c_master_write(cmd, (void*)out_data, out_size, true);
   if (err != ESP_OK)
      goto end;

   i2c_master_stop(cmd);
   err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT));
   if (err != ESP_OK)
      ESP_LOGE(TAG, "Could not write to device [0x%02x on port %d]: %d (%s)", dev->address, dev->port, err, esp_err_to_name(err));

end:
   i2c_cmd_link_delete(cmd);

   SEMAPHORE_GIVE(dev->port);
   return err;
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t* dev, uint8_t reg, const void* out_data, size_t out_size) {
   return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}

esp_err_t i2c_dev_read(const i2c_dev_t* dev, const void* out_data, size_t out_size, void* in_data, size_t in_size) {
   CHECK_ARG(dev && in_data);

   SEMAPHORE_TAKE(dev->port);

   if (!ports[dev->port].installed) {
      SEMAPHORE_GIVE(dev->port);
      return ESP_ERR_INVALID_STATE;
   }

   esp_err_t err = ESP_OK;

   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   assert(cmd != NULL);

   if (out_data && out_size) {
      err = i2c_master_start(cmd);
      if (err != ESP_OK)
         goto end;

      err = i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
      if (err != ESP_OK)
         goto end;

      err = i2c_master_write(cmd, (void*)out_data, out_size, true);
      if (err != ESP_OK)
         goto end;
   }

   err = i2c_master_start(cmd);
   if (err != ESP_OK)
      goto end;

   err = i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
   if (err != ESP_OK)
      goto end;

   err = i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
   if (err != ESP_OK)
      goto end;

   i2c_master_stop(cmd);

   err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT));
   if (err != ESP_OK)
      ESP_LOGE(TAG, "Could not read from device [0x%02x on port %d]: %d (%s)", dev->address, dev->port, err, esp_err_to_name(err));

end:
   i2c_cmd_link_delete(cmd);

   SEMAPHORE_GIVE(dev->port);
   return err;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t* dev, uint8_t reg, void* in_data, size_t in_size) {
   return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}
