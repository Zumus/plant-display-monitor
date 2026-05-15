#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "lvgl.h"

#define PIN_NUM_SCLK 39
#define PIN_NUM_MOSI 38
#define PIN_NUM_MISO 40

#define SPI_HOST SPI2_HOST

#define I2C_NUM 0 // I2C number
#define PIN_NUM_I2C_SDA 48
#define PIN_NUM_I2C_SCL 47

#define LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)

#define PIN_NUM_LCD_DC 42
#define PIN_NUM_LCD_RST -1
#define PIN_NUM_LCD_CS 45

#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

#define LCD_H_RES 240
#define LCD_V_RES 320

#define PIN_NUM_BK_LIGHT 1

#define LCD_BL_LEDC_TIMER LEDC_TIMER_0
#define LCD_BL_LEDC_MODE LEDC_LOW_SPEED_MODE

#define LCD_BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LCD_BL_LEDC_DUTY (1024) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LCD_BL_LEDC_FREQUENCY                                                  \
  (10000) // Frequency in Hertz. Set frequency at 5 kHz

#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1

#define SENSOR_DRY_MV 2200
#define SENSOR_WET_MV 950
#define BG_COLOR 0x2774AE
#define TEXT_COLOR 0xFFFFFF
#define FILL_COLOR 0xFFD100
static const char *TAG = "zmu_plant_monitor";
static lv_indev_drv_t input_driver;  // Input device driver (Touch)
static lv_disp_drv_t display_driver; /*Descriptor of a display driver*/
static SemaphoreHandle_t lvgl_api_mux = NULL;

esp_lcd_panel_handle_t panel_handle;
esp_lcd_touch_handle_t touch_handle;

lv_obj_t *label_brightness;
lv_obj_t *label_moisture_percent;
lv_obj_t *label_moisture_status;
lv_obj_t *bar_moisture;

lv_timer_t *brightness_timer = NULL;
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t my_adc_handle;
adc_cali_handle_t my_cali_handle;
lv_obj_t *label_voltage;

adc_oneshot_unit_handle_t initialize_adc() {
  // Initialize ADC unit
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  // Configure channel
  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12 // read up to 3.3V
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
  return adc1_handle;
}

int read_moisture_voltage(adc_oneshot_unit_handle_t adc_handle, adc_cali_handle_t calibration_handle) {
  int raw_value, voltage;
  ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &raw_value));
  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration_handle, raw_value, &voltage));
  return voltage;
}

adc_cali_handle_t initialize_calibration() {
  adc_cali_handle_t calibration_handle = NULL;
  adc_cali_curve_fitting_config_t calibration_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&calibration_config, &calibration_handle));
  return calibration_handle;
}

bool lvgl_lock(int timeout_ms) {
  // Convert timeout in milliseconds to FreeRTOS ticks
  // If `timeout_ms` is set to -1, the program will block until the condition
  // is met
  const TickType_t timeout_ticks =
      (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTakeRecursive(lvgl_api_mux, timeout_ticks) == pdTRUE;
}

const char* get_moisture_status(int percentage) {
    if (percentage >= 80) return "Overwatered";
    if (percentage >= 50) return "Water Later";
    if (percentage >= 20) return "Water Soon";
    return "Water Now";
}

int map_moisture_to_percent(int current_mv) {
    if (current_mv >= SENSOR_DRY_MV) return 0;
    if (current_mv <= SENSOR_WET_MV) return 100;

    int range = SENSOR_DRY_MV - SENSOR_WET_MV;
    int percentage = ((SENSOR_DRY_MV - current_mv) * 100) / range;
    
    return percentage;
}

void lvgl_moisture_ui_init(void) {
    lv_obj_t *screen = lv_scr_act();

    lv_obj_set_style_bg_color(screen, lv_color_hex(BG_COLOR), 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(TEXT_COLOR), 0);

    bar_moisture = lv_bar_create(screen);
    lv_obj_set_size(bar_moisture, 200, 30);
    lv_obj_align(bar_moisture, LV_ALIGN_CENTER, 0, 0);
    lv_bar_set_range(bar_moisture, 0, 100); // 0 to 100 percent
    lv_obj_set_style_bg_color(bar_moisture, lv_color_hex(FILL_COLOR), LV_PART_INDICATOR); 

    label_moisture_percent = lv_label_create(screen);
    lv_label_set_text(label_moisture_percent, "0%");
    lv_obj_align_to(label_moisture_percent, bar_moisture, LV_ALIGN_OUT_TOP_MID, 0, -10);

    label_moisture_status = lv_label_create(screen);
    lv_label_set_text(label_moisture_status, "Checking...");
    lv_obj_align_to(label_moisture_status, bar_moisture, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

void update_sensor_data_callback(lv_timer_t *timer) {
    int voltage = read_moisture_voltage(my_adc_handle, my_cali_handle);
    int percent = map_moisture_to_percent(voltage);
    const char* status_text = get_moisture_status(percent);
    ESP_LOGI(TAG, "Moisture: %d mV -> %d%%", voltage, percent);
    
    lv_bar_set_value(bar_moisture, percent, LV_ANIM_ON);
    lv_label_set_text_fmt(label_moisture_percent, "Moisture Level: %d%%", percent);
    lv_obj_align_to(label_moisture_percent, bar_moisture, LV_ALIGN_OUT_TOP_MID, 0, -10);
    lv_label_set_text(label_moisture_status, status_text);
    lv_obj_align_to(label_moisture_status, bar_moisture, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

void lvgl_unlock(void) { xSemaphoreGiveRecursive(lvgl_api_mux); }

static bool notify_lvgl_flush_ready(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *edata,
    void *user_ctx
) {
  lv_disp_flush_ready(&display_driver);
  return false;
}

static void increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_flush_callback(
    lv_disp_drv_t *drv,
    const lv_area_t *area,
    lv_color_t *color_map
) {
  int offset_x1 = area->x1;
  int offset_x2 = area->x2;
  int offset_y1 = area->y1;
  int offset_y2 = area->y2;
  // copy a buffer's content to a specific area of the display

  esp_lcd_panel_draw_bitmap(
      panel_handle,
      offset_x1,
      offset_y1,
      offset_x2 + 1,
      offset_y2 + 1,
      color_map
  );
}

static void lvgl_touch_callback(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  uint16_t touchpad_x[1] = {0};
  uint16_t touchpad_y[1] = {0};
  uint8_t touchpad_count = 0;
  esp_lcd_touch_read_data(touch_handle);

  bool touchpad_pressed = esp_lcd_touch_get_coordinates(
      touch_handle,
      touchpad_x,
      touchpad_y,
      NULL,
      &touchpad_count,
      1
  );

  if (touchpad_pressed && touchpad_count > 0) {
    data->point.x = touchpad_x[0];
    data->point.y = touchpad_y[0];
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void lv_port_display_init(void) {
  static lv_disp_draw_buf_t draw_buffer;
  lv_color_t *frame_buffer_1 = heap_caps_malloc(
      LCD_H_RES * LCD_V_RES * sizeof(lv_color_t),
      MALLOC_CAP_SPIRAM
  );
  assert(frame_buffer_1);
  lv_color_t *frame_buffer_2 = heap_caps_malloc(
      LCD_H_RES * LCD_V_RES * sizeof(lv_color_t),
      MALLOC_CAP_SPIRAM
  );
  assert(frame_buffer_2);

  /*Initialize the display buffer*/
  lv_disp_draw_buf_init(
      &draw_buffer,
      frame_buffer_1,
      frame_buffer_2,
      LCD_H_RES * LCD_V_RES
  );

  /*-----------------------------------
   * Register the display in LVGL
   *----------------------------------*/

  lv_disp_drv_init(&display_driver);

  /*Set the resolution of the display*/
  display_driver.hor_res = LCD_H_RES;
  display_driver.ver_res = LCD_V_RES;

  /*Used to copy the buffer's content to the display*/
  display_driver.flush_cb = lvgl_flush_callback;
  display_driver.draw_buf = &draw_buffer;
  display_driver.full_refresh = 1;
  lv_disp_drv_register(&display_driver);
}

void lv_port_input_device_init(void) {
  lv_indev_drv_init(&input_driver);
  input_driver.type = LV_INDEV_TYPE_POINTER;
  // input_driver.disp = disp;
  input_driver.read_cb = lvgl_touch_callback;
  input_driver.user_data = touch_handle;

  lv_indev_drv_register(&input_driver);
}

void display_init(void) {
  ESP_LOGI(TAG, "SPI BUS init");
  spi_bus_config_t buscfg = {
      .sclk_io_num = PIN_NUM_SCLK,
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = PIN_NUM_MISO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4000,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(TAG, "Install panel IO");

  esp_lcd_panel_io_handle_t io_handle = NULL;

  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = PIN_NUM_LCD_DC,
      .cs_gpio_num = PIN_NUM_LCD_CS,
      .pclk_hz = LCD_PIXEL_CLOCK_HZ,
      .lcd_cmd_bits = LCD_CMD_BITS,
      .lcd_param_bits = LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
      .on_color_trans_done = notify_lvgl_flush_ready,
  };
  // Attach the LCD to the SPI bus
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
      (esp_lcd_spi_bus_handle_t)SPI_HOST,
      &io_config,
      &io_handle
  ));

  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_NUM_LCD_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel = 16,
  };
  ESP_LOGI(TAG, "Install ST7789 panel driver");
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle)
  );

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
}

void touch_init(void) {
  esp_lcd_panel_io_handle_t tp_io_handle = NULL;

  ESP_LOGI(TAG, "Initialize I2C");
  const i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = PIN_NUM_I2C_SDA,
      .scl_io_num = PIN_NUM_I2C_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 400000,
  };
  /* Initialize I2C */
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, i2c_conf.mode, 0, 0, 0));

  ESP_LOGI(TAG, "Initialize touch IO (I2C)");
  esp_lcd_panel_io_i2c_config_t tp_io_config =
      ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(
      (esp_lcd_i2c_bus_handle_t)I2C_NUM,
      &tp_io_config,
      &tp_io_handle
  ));

  esp_lcd_touch_config_t tp_cfg = {
      .x_max = LCD_V_RES,
      .y_max = LCD_H_RES,
      .rst_gpio_num = -1,
      .int_gpio_num = -1,
      .flags = {
          .swap_xy = 0,
          .mirror_x = 0,
          .mirror_y = 0,
      },
  };

  ESP_LOGI(TAG, "Initialize touch controller CST816");
  ESP_ERROR_CHECK(
      esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle)
  );
}

void backlight_init(void) {
  gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_NUM_BK_LIGHT, 1);

  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LCD_BL_LEDC_MODE,
      .timer_num = LCD_BL_LEDC_TIMER,
      .duty_resolution = LCD_BL_LEDC_DUTY_RES,
      .freq_hz = LCD_BL_LEDC_FREQUENCY, // Set output frequency at 5 kHz
      .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {
      .speed_mode = LCD_BL_LEDC_MODE,
      .channel = LCD_BL_LEDC_CHANNEL,
      .timer_sel = LCD_BL_LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = PIN_NUM_BK_LIGHT,
      .duty = 0, // Set duty to 0%
      .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void backlight_set_level(uint8_t level) {
  if (level > 100) {
    ESP_LOGE(TAG, "Brightness value out of range");
    return;
  }

  uint32_t duty = (level * (LCD_BL_LEDC_DUTY - 1)) / 100;

  ESP_ERROR_CHECK(ledc_set_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL));

  ESP_LOGI(TAG, "LCD brightness set to %d%%", level);
}

void lvgl_tick_timer_init(uint32_t ms) {
  ESP_LOGI(TAG, "Install LVGL tick timer");
  // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &increase_lvgl_tick,
      .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, ms * 1000));
}

static void run_lvgl(void *param) {
  // ESP_LOGI(TAG, "run");
  while (1) {
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
      // Lock the mutex due to the LVGL APIs are not thread-safe
      if (lvgl_lock(-1)) {
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_unlock();
      }
      if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
        task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
      } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
        task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
      }
      vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
  }
}



void slider_event_callback(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    lv_obj_t *slider = lv_event_get_target(e);
    int value = lv_slider_get_value(slider);
    // printf("Slider value: %d\n", value);

    lv_label_set_text_fmt(label_brightness, "%d %%", value);
    backlight_set_level(value);
    lv_event_stop_bubbling(e);
  }
}

void lvgl_brightness_ui_init(lv_obj_t *parent) {
  lv_obj_t *obj = lv_obj_create(parent);
  lv_obj_set_size(obj, lv_pct(90), lv_pct(50));
  lv_obj_align(obj, LV_ALIGN_CENTER, 0, 0);
  lv_obj_t *slider = lv_slider_create(obj);

  lv_slider_set_range(slider, 1, 100);
  lv_slider_set_value(slider, 80, LV_ANIM_OFF);

  lv_obj_set_size(slider, lv_pct(90), 20);
  lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_style_pad_top(obj, 20, 0);
  lv_obj_set_style_pad_bottom(obj, 20, 0);
  // lv_obj_set_style_pad_left(parent, 50, 0);
  // lv_obj_set_style_pad_right(parent, 50, 0);
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE);
  lv_obj_add_event_cb(
      slider,
      slider_event_callback,
      LV_EVENT_VALUE_CHANGED,
      NULL
  );

  label_brightness = lv_label_create(obj);
  lv_label_set_text(label_brightness, "80%");
  lv_obj_align(label_brightness, LV_ALIGN_TOP_MID, 0, 0);
  label_voltage = lv_label_create(parent);
  lv_label_set_text(label_voltage, "Moisture: Reading...");
  lv_obj_align(label_voltage, LV_ALIGN_BOTTOM_MID, 0, -50);
}

void app_main(void) {
  my_adc_handle = initialize_adc();
  my_cali_handle = initialize_calibration();
  lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
  lv_init();
  display_init();
  touch_init();
  lv_port_display_init();
  lv_port_input_device_init();
  lvgl_tick_timer_init(LVGL_TICK_PERIOD_MS);
  backlight_init();
  backlight_set_level(80);
  if (lvgl_lock(-1)) {
    // lvgl_brightness_ui_init(lv_scr_act());
    lvgl_moisture_ui_init();
    lv_timer_create(update_sensor_data_callback, 500, NULL);
    lvgl_unlock();
  }
  xTaskCreatePinnedToCore(run_lvgl, "lvgl_task", 1024 * 20, NULL, 5, NULL, 1);
}
