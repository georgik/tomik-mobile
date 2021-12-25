/* Camera Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)
    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "lcd.h"
#include "esp_camera.h"
#include "board.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/rmt.h"
#include "led_strip.h"
#include "i2c_bus.h"
#include "audio.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO 8
#define PIN_NUM_MOSI 9
#define PIN_NUM_CLK  15
#define PIN_NUM_CS   40
#define SPI_DMA_CHAN    host.slot

int previous_state = 0;
int lock = 0;

static const char *TAG = "main";
#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1<<GPIO_OUTPUT_IO_0) | (1<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     2
#define GPIO_INPUT_IO_1     14
#define GPIO_INPUT_IO_2     36
#define GPIO_INPUT_IO_3     37
#define GPIO_INPUT_PIN_SEL  ((1<<GPIO_INPUT_IO_0) | (1<<GPIO_INPUT_IO_1) | (1<<GPIO_INPUT_IO_2) | (1<<GPIO_INPUT_IO_3))
#define ESP_INTR_FLAG_DEFAULT 0

#define IMAGE_MAX_SIZE (100 * 1024)/**< The maximum size of a single picture in the boot animation */
#define IMAGE_WIDTH    320 /*!< width of jpeg file */
#define IMAGE_HIGHT    240 /*!< height of jpeg file */


static xQueueHandle gpio_evt_queue = NULL;

void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


/**
 * @brief rgb -> rgb565
 *
 * @param r red   (0~31)
 * @param g green (0~63)
 * @param b red   (0~31)
 *
 * @return data about color565
 */
uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t color = ((r  << 11) | (g  << 6) | b);
    return (color << 8) | (color >> 8);
}

void init_spiff() {
        esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 20,
        .format_if_mount_failed = false
    };

    /*!< Use settings defined above to initialize and mount SPIFFS filesystem. */
    /*!< Note: esp_vfs_spiffs_register is an all-in-one convenience function. */
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    size_t total = 0, used = 0;
    ESP_ERROR_CHECK(esp_spiffs_info(NULL, &total, &used));

}



void esp_photo_display(void)
{
    ESP_LOGI(TAG, "LCD photo test....");
    if (lock == 1) {return;}
    lock=1;
    uint8_t *rgb565 = malloc(IMAGE_WIDTH * IMAGE_HIGHT * 2);
    if (NULL == rgb565) {
        ESP_LOGE(TAG, "can't alloc memory for rgb565 buffer");
        return;
    }
    uint8_t *buf = malloc(IMAGE_MAX_SIZE);
    if (NULL == buf) {
        free(rgb565);
        ESP_LOGE(TAG, "can't alloc memory for jpeg file buffer");
        return;
    }
    int read_bytes = 0;

    FILE *fd = fopen("/spiffs/image.jpg", "r");

    read_bytes = fread(buf, 1, IMAGE_MAX_SIZE, fd);
    ESP_LOGI(TAG, "spiffs:read_bytes:%d  fd: %p", read_bytes, fd);
    fclose(fd);

    jpg2rgb565(buf, read_bytes, rgb565, JPG_SCALE_NONE);
    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data(rgb565, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
    free(buf);
    free(rgb565);
    vTaskDelay(500 / portTICK_RATE_MS);
    lock = 0;
    // vTaskDelay(2000 / portTICK_RATE_MS);
}


void esp_pear_display(void)
{
    ESP_LOGI(TAG, "LCD photo test....");
    if (lock == 1) {return;}
    lock=1;
    uint8_t *rgb565 = malloc(IMAGE_WIDTH * IMAGE_HIGHT * 2);
    if (NULL == rgb565) {
        ESP_LOGE(TAG, "can't alloc memory for rgb565 buffer");
        return;
    }
    uint8_t *buf = malloc(IMAGE_MAX_SIZE);
    if (NULL == buf) {
        free(rgb565);
        ESP_LOGE(TAG, "can't alloc memory for jpeg file buffer");
        return;
    }
    int read_bytes = 0;

    FILE *fd = fopen("/sdcard/img/1.jpg", "r");

    read_bytes = fread(buf, 1, IMAGE_MAX_SIZE, fd);
    ESP_LOGI(TAG, "spiffs:read_bytes:%d  fd: %p", read_bytes, fd);
    fclose(fd);

    jpg2rgb565(buf, read_bytes, rgb565, JPG_SCALE_NONE);
    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data(rgb565, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
    free(buf);
    free(rgb565);
    vTaskDelay(500 / portTICK_RATE_MS);
    lock = 0;
    // vTaskDelay(2000 / portTICK_RATE_MS);
}

void esp_barbapapa_display(void)
{
    ESP_LOGI(TAG, "LCD photo test....");
    if (lock == 1) {return;}
    lock=1;
    uint8_t *rgb565 = malloc(IMAGE_WIDTH * IMAGE_HIGHT * 2);
    if (NULL == rgb565) {
        ESP_LOGE(TAG, "can't alloc memory for rgb565 buffer");
        return;
    }
    uint8_t *buf = malloc(IMAGE_MAX_SIZE);
    if (NULL == buf) {
        free(rgb565);
        ESP_LOGE(TAG, "can't alloc memory for jpeg file buffer");
        return;
    }
    int read_bytes = 0;

    FILE *fd = fopen("/sdcard/img/2.jpg", "r");

    read_bytes = fread(buf, 1, IMAGE_MAX_SIZE, fd);
    ESP_LOGI(TAG, "spiffs:read_bytes:%d  fd: %p", read_bytes, fd);
    fclose(fd);

    jpg2rgb565(buf, read_bytes, rgb565, JPG_SCALE_NONE);
    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data(rgb565, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
    free(buf);
    free(rgb565);
    vTaskDelay(500 / portTICK_RATE_MS);
    lock = 0;
    // vTaskDelay(2000 / portTICK_RATE_MS);
}


void esp_power_display(void)
{
    ESP_LOGI(TAG, "LCD photo test....");
    if (lock == 1) {return;}
    lock=1;
    uint8_t *rgb565 = malloc(IMAGE_WIDTH * IMAGE_HIGHT * 2);
    if (NULL == rgb565) {
        ESP_LOGE(TAG, "can't alloc memory for rgb565 buffer");
        return;
    }
    uint8_t *buf = malloc(IMAGE_MAX_SIZE);
    if (NULL == buf) {
        free(rgb565);
        ESP_LOGE(TAG, "can't alloc memory for jpeg file buffer");
        return;
    }
    int read_bytes = 0;

    FILE *fd = fopen("/sdcard/img/3.jpg", "r");

    read_bytes = fread(buf, 1, IMAGE_MAX_SIZE, fd);
    ESP_LOGI(TAG, "spiffs:read_bytes:%d  fd: %p", read_bytes, fd);
    fclose(fd);

    jpg2rgb565(buf, read_bytes, rgb565, JPG_SCALE_NONE);
    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data(rgb565, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
    free(buf);
    free(rgb565);
    vTaskDelay(500 / portTICK_RATE_MS);
    lock = 0;
    // vTaskDelay(2000 / portTICK_RATE_MS);
}



void esp_pink_display(void)
{
    ESP_LOGI(TAG, "LCD photo test....");
    if (lock == 1) {return;}
    lock=1;
    uint8_t *rgb565 = malloc(IMAGE_WIDTH * IMAGE_HIGHT * 2);
    if (NULL == rgb565) {
        ESP_LOGE(TAG, "can't alloc memory for rgb565 buffer");
        return;
    }
    uint8_t *buf = malloc(IMAGE_MAX_SIZE);
    if (NULL == buf) {
        free(rgb565);
        ESP_LOGE(TAG, "can't alloc memory for jpeg file buffer");
        return;
    }
    int read_bytes = 0;

    FILE *fd = fopen("/sdcard/img/4.jpg", "r");

    read_bytes = fread(buf, 1, IMAGE_MAX_SIZE, fd);
    ESP_LOGI(TAG, "spiffs:read_bytes:%d  fd: %p", read_bytes, fd);
    fclose(fd);

    jpg2rgb565(buf, read_bytes, rgb565, JPG_SCALE_NONE);
    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data(rgb565, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
    free(buf);
    free(rgb565);
    vTaskDelay(500 / portTICK_RATE_MS);
    lock = 0;
    // vTaskDelay(2000 / portTICK_RATE_MS);
}

void esp_photo_display2(void)
{
//     ESP_LOGI(TAG, "LCD photo test....");
//     esp_vfs_spiffs_conf_t conf = {
//         .base_path = "/spiffs",
//         .partition_label = NULL,
//         .max_files = 5,
//         .format_if_mount_failed = false
//     };

//     /*!< Use settings defined above to initialize and mount SPIFFS filesystem. */
//     /*!< Note: esp_vfs_spiffs_register is an all-in-one convenience function. */
//     ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
//     size_t total = 0, used = 0;
//     ESP_ERROR_CHECK(esp_spiffs_info(NULL, &total, &used));
    if (lock==1) {return;}
    lock = 1;
    uint8_t *rgb565 = malloc(IMAGE_WIDTH * IMAGE_HIGHT * 2);
    if (NULL == rgb565) {
        ESP_LOGE(TAG, "can't alloc memory for rgb565 buffer");
        return;
    }
    uint8_t *buf = malloc(IMAGE_MAX_SIZE);
    if (NULL == buf) {
        free(rgb565);
        ESP_LOGE(TAG, "can't alloc memory for jpeg file buffer");
        return;
    }
    int read_bytes = 0;

    FILE *fd = fopen("/sdcard/img/5.jpg", "r");

    read_bytes = fread(buf, 1, IMAGE_MAX_SIZE, fd);
    ESP_LOGI(TAG, "spiffs:read_bytes:%d  fd: %p", read_bytes, fd);
    fclose(fd);

    jpg2rgb565(buf, read_bytes, rgb565, JPG_SCALE_NONE);
    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data(rgb565, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
    free(buf);
    free(rgb565);
    vTaskDelay(500 / portTICK_RATE_MS);
    lock = 0;
    // vTaskDelay(2000 / portTICK_RATE_MS);
}
void button_on() {
    ESP_LOGI(TAG, "LCD color test....");
    uint16_t *data_buf = (uint16_t *)heap_caps_calloc(IMAGE_WIDTH * IMAGE_HIGHT, sizeof(uint16_t), MALLOC_CAP_SPIRAM);

    uint16_t color = color565(0, 0, 0);

    for (int r = 0,  j = 0; j < IMAGE_HIGHT; j++) {
        if (j % 8 == 0) {
            color = color565(r++, 0, 0);
        }

        for (int i = 0; i < IMAGE_WIDTH; i++) {
            data_buf[i + IMAGE_WIDTH * j] = color;
        }
    }

    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data((uint8_t *)data_buf, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));
}

void button_off() {
    ESP_LOGI(TAG, "LCD color test....");

    uint16_t color = color565(0, 0, 0);
    uint16_t *data_buf = (uint16_t *)heap_caps_calloc(IMAGE_WIDTH * IMAGE_HIGHT, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        for (int b = 0,  j = 0; j < IMAGE_HIGHT; j++) {
            if (j % 8 == 0) {
                color = color565(0, 0, b++);
            }

            for (int i = 0; i < IMAGE_WIDTH; i++) {
                data_buf[i + IMAGE_WIDTH * j] = color;
            }
        }

        lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
        lcd_write_data((uint8_t *)data_buf, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));

}

void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            // up - down
            // if (io_num == GPIO_INPUT_IO_3) {
                int level2 = adc1_get_raw(ADC1_CHANNEL_1);
                printf("up-down%d\n", level2);
            //     continue;
            // } else if (io_num == GPIO_INPUT_IO_2)
            // {
                int level3 = adc1_get_raw(ADC1_CHANNEL_0);
                printf("left-right%d\n", level3);

            //     continue;
            // }

            int level = gpio_get_level(io_num);
           
            printf("GPIO[%d] intr, val: %d\n", io_num, level);
            
            if (previous_state == level) {
                continue;
            }
            

            if (level == 1) {
                esp_photo_display2();
            } else {
                esp_photo_display();
            }
            previous_state = level;
        }
    }
    
}


void esp_color_display(int color)
{
    ESP_LOGI(TAG, "LCD color test....");
    uint16_t *data_buf = (uint16_t *)heap_caps_calloc(IMAGE_WIDTH * IMAGE_HIGHT, sizeof(uint16_t), MALLOC_CAP_SPIRAM);


    for (int r = 0,  j = 0; j < IMAGE_HIGHT; j++) {
        // if (j % 8 == 0) {
        //     color = color565(r++, 0, 0);
        // }

        for (int i = 0; i < IMAGE_WIDTH; i++) {
            data_buf[i + IMAGE_WIDTH * j] = color;
        }
    }

    lcd_set_index(0, 0, IMAGE_WIDTH - 1, IMAGE_HIGHT - 1);
    lcd_write_data((uint8_t *)data_buf, IMAGE_WIDTH * IMAGE_HIGHT * sizeof(uint16_t));

}




#define DEFAULT_VREF    1100                            /*!< Use adc2_vref_to_gpio() to obtain a better estimate */
#define NO_OF_SAMPLES   64
#define SAMPLE_TIME     200                             /*!< Sampling time(ms) */
static const adc_channel_t channel = ADC_CHANNEL_5;     /*!< PIO7 if ADC1, GPIO17 if ADC2 */
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static xQueueHandle adc_queue = NULL;
led_strip_t *strip;
#define DEVIATION 0.1
#define LED_MAX_VALUE 5

double adc_voltage_conversion(uint32_t adc_reading)
{
    double voltage = 0;

    voltage = (2.60 * adc_reading) / 8191;

    return voltage;
}
void button_task(void *arg)
{
    /*!<Continuously sample ADC1*/
    while (1) {
        uint32_t adc_reading = 0;
        double voltage = 0;

        /*!< Multisampling */
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }

        adc_reading /= NO_OF_SAMPLES;

        voltage = adc_voltage_conversion(adc_reading);
        ESP_LOGD(TAG, "ADC%d CH%d Raw: %d   ; Voltage: %0.2lfV", unit, channel, adc_reading, voltage);

        xQueueSend(adc_queue, (double *)&voltage, 0);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME));
    }

    vTaskDelete(NULL);
}

void led_task(void *arg)
{
    double voltage = 0;

    while (1) {
        xQueueReceive(adc_queue, &voltage, portMAX_DELAY);

        if (voltage > 2.6) {
            continue;
        } else if (voltage > 2.41 - DEVIATION  && voltage <= 2.41 + DEVIATION) {
            ESP_LOGI(TAG, "rec(K1) -> red");
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, LED_MAX_VALUE, 0, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 0));
            esp_barbapapa_display();
            // esp_color_display(color565(255,0,0));
        } else if (voltage > 1.98 - DEVIATION && voltage <= 1.98 + DEVIATION) {
            esp_pear_display();
            ESP_LOGI(TAG, "mode(K2) -> green");
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, LED_MAX_VALUE, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 0));
            // esp_color_display(color565(0,255,0));
        } else if (voltage > 1.65 - DEVIATION && voltage <= 1.65 + DEVIATION) {
            ESP_LOGI(TAG, "play(K3) -> blue");
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, 0, LED_MAX_VALUE));
            ESP_ERROR_CHECK(strip->refresh(strip, 0));
            esp_color_display(color565(0,0,255));
        } else if (voltage > 1.11 - DEVIATION && voltage <= 1.11 + DEVIATION) {
            ESP_LOGI(TAG, "set(K4) -> yellow");
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, LED_MAX_VALUE, LED_MAX_VALUE, 0));
            ESP_ERROR_CHECK(strip->refresh(strip, 0));
            esp_power_display();
            aplay_mp3("/sdcard/Audio/3.mp3");
            // esp_color_display(color565(255,255,0));
        } else if (voltage > 0.82 - DEVIATION && voltage <= 0.82 + DEVIATION) {
            ESP_LOGI(TAG, "vol(K5) -> purple");
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, LED_MAX_VALUE, 0, LED_MAX_VALUE));
            ESP_ERROR_CHECK(strip->refresh(strip, 0));
            esp_pink_display();
            aplay_mp3("/sdcard/Audio/2.mp3");
            // esp_color_display(color565(255,0,255));
        } else if (voltage > 0.38 - DEVIATION && voltage <= 0.38 + DEVIATION) {
            ESP_LOGI(TAG, "vol+(K6) -> write");
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, LED_MAX_VALUE, LED_MAX_VALUE, LED_MAX_VALUE));
            ESP_ERROR_CHECK(strip->refresh(strip, 0));
            esp_color_display(color565(255,255,255));
            aplay_mp3("/sdcard/Audio/1.mp3");
        }

    }

    vTaskDelete(NULL);

}

void adc_init(void)
{
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

}

esp_err_t example_rmt_init(uint8_t gpio_num, int led_number, uint8_t rmt_channel)
{
    ESP_LOGI(TAG, "Initializing WS2812");
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio_num, rmt_channel);

    /*!< set counter clock to 40MHz */
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(led_number, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);

    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
        return ESP_FAIL;
    }

    /*!< Clear LED strip (turn off all LEDs) */
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    /*!< Show simple rainbow chasing pattern */

    return ESP_OK;
}




void app_main()
{
    lcd_config_t lcd_config = {
#ifdef CONFIG_LCD_ST7789
        .clk_fre         = 80 * 1000 * 1000, /*!< ILI9341 Stable frequency configuration */
#endif
#ifdef CONFIG_LCD_ILI9341
        .clk_fre         = 40 * 1000 * 1000, /*!< ILI9341 Stable frequency configuration */
#endif
        .pin_clk         = LCD_CLK,
        .pin_mosi        = LCD_MOSI,
        .pin_dc          = LCD_DC,
        .pin_cs          = LCD_CS,
        .pin_rst         = LCD_RST,
        .pin_bk          = LCD_BK,
        .max_buffer_size = 2 * 1024,
        .horizontal      = 2, /*!< 2: UP, 3: DOWN */
        .swap_data       = 1,
    };

    lcd_init(&lcd_config);
init_spiff();
    /*< Show a picture */
    esp_photo_display();

/* Single button controller 
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode        
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    */
/*
//change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_ANYEDGE);
    // gpio_set_intr_type(GPIO_INPUT_IO_2, GPIO_INTR_ANYEDGE);
    // gpio_set_intr_type(GPIO_INPUT_IO_3, GPIO_INTR_ANYEDGE);
        // adc1_config_width(ADC_WIDTH_BIT_13);
        // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
        // adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
        printf("adc ready\n");
     //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    printf("task created\n");
      //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    // gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
    // gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
*/

    ESP_ERROR_CHECK(example_rmt_init(CONFIG_EXAMPLE_RMT_TX_GPIO, CONFIG_EXAMPLE_STRIP_LED_NUMBER, RMT_CHANNEL_0));

    adc_queue = xQueueCreate(1, sizeof(double));
    adc_init();

    ESP_ERROR_CHECK(i2c_bus_init());
    audio_init_simple();

    xTaskCreatePinnedToCore(&button_task, "button_task", 3 * 1024, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&led_task, "led_task", 3 * 1024, NULL, 5, NULL, 0);

esp_err_t ret;
   esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
       ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

   sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

     sdmmc_card_print_info(stdout, card);    

    /*< RGB display */
    //esp_color_display();

}

