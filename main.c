#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "dht.h"
#include "bh1750.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "cJSON.h"

#define BH1750_ADDR BH1750_ADDR_LO
#define I2C_MASTER_SDA GPIO_NUM_21
#define I2C_MASTER_SCL GPIO_NUM_22
#define DHT_GPIO_PIN GPIO_NUM_5

static const char *TAG1 = "AnalogSensor";
static const char *TAG2 = "DigitalSensor";
static const char *TAG3 = "CommunicateSensor";
static const char *TAG_MQTT = "MQTT_Client";

// Sensor Data Variables
static float temperature = 0.0f, humidity = 0.0f;
static uint16_t light_intensity = 0;
static int Gas_level = 0;

SemaphoreHandle_t sensor_mutex;
esp_mqtt_client_handle_t mqtt_client;
bool mqtt_connected = false;

void BH1750Task(void *pvParameters) {
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));
    if (bh1750_init_desc(&dev, BH1750_ADDR, 0, I2C_MASTER_SDA, I2C_MASTER_SCL) != ESP_OK) {
        ESP_LOGE(TAG3, "Failed to initialize BH1750");
        vTaskDelete(NULL);
    }
    bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH);
    while (1) {
        uint16_t lux;
        if (bh1750_read(&dev, &lux) == ESP_OK) {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            light_intensity = lux;
            xSemaphoreGive(sensor_mutex);
            ESP_LOGI(TAG3, "Light Intensity: %d lux", lux);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void AnalogSensorTask(void *params) {
    // Initialize ADC
    esp_adc_cal_characteristics_t adc1_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    while (1) {
        // Read ADC value
        int adc_value = adc1_get_raw(ADC1_CHANNEL_4);
        if (adc_value >= 0) { // Check if the value is valid
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            Gas_level = adc_value;
            xSemaphoreGive(sensor_mutex);
            ESP_LOGI(TAG1, "Gas Level: %d", adc_value);
        } else {
            ESP_LOGE(TAG1, "Failed to read ADC value");
        }

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void DHTTask(void *pvParameters) {
    while (1) {
        float temp, hum;
        if (dht_read_float_data(DHT_TYPE_AM2301, DHT_GPIO_PIN, &hum, &temp) == ESP_OK) {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            temperature = temp;
            humidity = hum;
            xSemaphoreGive(sensor_mutex);
            ESP_LOGI(TAG2, "Temperature: %.1fC, Humidity: %.1f%%", temp, hum);
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void mqtt_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    esp_mqtt_event_t *event = (esp_mqtt_event_t *)event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT Connected");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "MQTT Disconnected");
            mqtt_connected = false;
            break;
        default:
            break;
    }
}

void mqtt_publish_task(void *arg) {
    while (1) {
        if (mqtt_connected) {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            cJSON *json = cJSON_CreateObject();
            cJSON_AddNumberToObject(json, "temperature", temperature);
            cJSON_AddNumberToObject(json, "humidity", humidity);
            cJSON_AddNumberToObject(json, "light_intensity", light_intensity);
            cJSON_AddNumberToObject(json, "Gas_level", Gas_level);
            char *json_str = cJSON_PrintUnformatted(json);
            cJSON_Delete(json);
            xSemaphoreGive(sensor_mutex);
            esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", json_str, 0, 1, 0);
            free(json_str);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    sensor_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (i2cdev_init() != ESP_OK)
    {
        ESP_LOGE(TAG3, "Failed to initialize I2C driver");
        return;
    }
    while (example_connect() != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "WiFi Connection Failed, Retrying...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    esp_mqtt_client_config_t mqtt_client_config = {
        .broker.address.uri = "mqtt://demo.thingsboard.io",
        .broker.address.port = 1883,
        .credentials.username = "rFXfWh5A4vzLFkaovYX3"
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_client_config);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    xTaskCreate(BH1750Task, "BH1750_Task", 2048, NULL, 5, NULL);
    xTaskCreate(AnalogSensorTask, "Analog_Sensor_Task", 2048, NULL, 6, NULL);
    xTaskCreate(DHTTask, "DHT_Task", 2048, NULL, 5, NULL);
    xTaskCreate(mqtt_publish_task, "MQTT_Publish_Task", 4096, NULL, 5, NULL);
}
