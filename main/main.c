
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "vehicledata.h"

#define tag "SSD1306"

#define ENCODER_PIN_A 26
#define ENCODER_PIN_B 27
#define BUTTON_PIN 25
#define GPIO_BIT_MASK_ENCODER  ((1ULL<<GPIO_NUM_26) | (1ULL<<GPIO_NUM_27)) 
#define GPIO_BIT_MASK_BUTTON  (1ULL<<GPIO_NUM_25)


#define SIGNAL_RPM_PIN 18 //12
#define SIGNAL_WHEEL_PIN 19 //14
#define GPIO_BIT_MASK_SIGNAL  ((1ULL<<GPIO_NUM_18) | (1ULL<<GPIO_NUM_19)) 
//Test pins for input signals
#define GPIO_BIT_MASK_SIGNALTEST  ((1ULL<<GPIO_NUM_12) | (1ULL<<GPIO_NUM_14)) 


#define SPEED_MODE 0
#define RPM_MODE   1

int enginespeed = 0;
int old_enginespeed = -1;
int vehiclespeed = 0;
int old_vehiclespeed = -1;
int mode = RPM_MODE;
int old_mode = RPM_MODE;
int encoderPinALast = LOW;
int encoderPinANow = LOW;
unsigned long debounce_button = 0;
int debounce_time_button = 200;
unsigned long debounce_encoder = 0;
int debounce_time_encoder = 1;
int incSpeed = 5;
int incRPM = 500;
int maxSpeed = 300;
int maxRPM = 16000;
int encoderPos = 0;
int counter = 120;
//int prescaler_speed = 2;
uint32_t coreFrequency = 40000000;
gptimer_handle_t gptimer_rpm = NULL;
gptimer_handle_t gptimer_speed = NULL;


static void encoder_interrupt_handler(void *args)
{
  if((xTaskGetTickCount()-debounce_encoder)>debounce_time_encoder)
  {
    encoderPinANow = gpio_get_level(ENCODER_PIN_A);
    if ((encoderPinALast == HIGH) && (encoderPinANow == LOW)) {
      if (gpio_get_level(ENCODER_PIN_B) == HIGH) {
        encoderPos = 1;
      } else {
        encoderPos = -1;
      }
    }
    encoderPinALast = encoderPinANow;
    if(mode == SPEED_MODE)
    {
      vehiclespeed = vehiclespeed + incSpeed * encoderPos;
      if(vehiclespeed > maxSpeed)
        vehiclespeed = maxSpeed;
      if (vehiclespeed < 0)
        vehiclespeed = 0;
//      calcSpeedFrequency();
    }
    else
    {
      enginespeed = enginespeed + incRPM * encoderPos;
      if(enginespeed > maxRPM)
        enginespeed = maxRPM;
      if(enginespeed < 0)
        enginespeed = 0;
//      calcRPMFrequency();
    }
    debounce_encoder=xTaskGetTickCount();
  }

}

static void button_interrupt_handler(void *args)
{
  if((xTaskGetTickCount()-debounce_button)>debounce_time_button)
  {
    if(mode == RPM_MODE)
      mode = SPEED_MODE;
    else
      mode = RPM_MODE;
    debounce_button = xTaskGetTickCount();
  }
}

static bool rpm_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
  gpio_set_level(SIGNAL_RPM_PIN,crankValue[counter]);
  counter--;
  if(counter==0)
    counter=119;
  return pdTRUE;
}

static bool speed_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
  gpio_set_level(SIGNAL_WHEEL_PIN, !gpio_get_level(SIGNAL_WHEEL_PIN));
  return pdTRUE;
}


static void rpm_timer_init()
{
	gptimer_config_t timer_config = {
    	.clk_src = GPTIMER_CLK_SRC_DEFAULT,
    	.direction = GPTIMER_COUNT_UP,
    	.resolution_hz = coreFrequency, 
	};
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_rpm));

//	gptimer_alarm_config_t alarm_config = {
//        .flags.auto_reload_on_alarm = 1,
//		.alarm_count = 800000,
//    };
//	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_rpm, &alarm_config));

	gptimer_event_callbacks_t cbs = {
    	.on_alarm = rpm_timer_isr, // register user callback
}	;
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_rpm, &cbs, (void*) NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer_rpm));
    ESP_ERROR_CHECK(gptimer_stop(gptimer_rpm));
}

static void speed_timer_init()
{
	gptimer_config_t timer_config = {
    	.clk_src = GPTIMER_CLK_SRC_DEFAULT,
    	.direction = GPTIMER_COUNT_UP,
    	.resolution_hz = coreFrequency, 
	};
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_speed));

//	gptimer_alarm_config_t alarm_config = {
//        .flags.auto_reload_on_alarm = 1,
//		.alarm_count = 800000,
//    };
//	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_speed, &alarm_config));

	gptimer_event_callbacks_t cbs = {
    	.on_alarm = speed_timer_isr, // register user callback
}	;
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_rpm, &cbs, (void*) NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer_rpm));
    ESP_ERROR_CHECK(gptimer_stop(gptimer_rpm));
}



void setup(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
//configure encoder pins
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_BIT_MASK_ENCODER;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
//configure button pin
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
 	io_conf.pin_bit_mask = GPIO_BIT_MASK_BUTTON; 
    gpio_config(&io_conf);

//configure signal pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_BIT_MASK_SIGNAL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

//configure test pins for speed and rpm signal
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_BIT_MASK_SIGNALTEST;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
	gpio_isr_handler_add(ENCODER_PIN_A, encoder_interrupt_handler, (void*)ENCODER_PIN_A);
	gpio_isr_handler_add(ENCODER_PIN_B, encoder_interrupt_handler, (void*)ENCODER_PIN_B);
	gpio_isr_handler_add(BUTTON_PIN, button_interrupt_handler, (void*)BUTTON_PIN);

	rpm_timer_init();
	speed_timer_init();
}

void calcRPMFrequency()
{
  int f;
  uint64_t tf;
  if(enginespeed > 0)
  {
    f=enginespeed*2;
    tf=coreFrequency/f-1;
	gptimer_alarm_config_t alarm_config = {
        .flags.auto_reload_on_alarm = 1,
		.alarm_count = tf,
    };
	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_rpm, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer_rpm));
  }
  else
  {
    ESP_ERROR_CHECK(gptimer_stop(gptimer_rpm));
  }
}

void calcSpeedFrequency()
{
  float speedms; 
  float ff;
  float f;
  uint64_t tf;
  if(vehiclespeed > 0)
  {
    speedms = vehiclespeed/3.6;
    ff = speedms/rollingCircumference*pulsesPerRotation;
    f = 2*ff;
    tf=round(coreFrequency/f-1);
	gptimer_alarm_config_t alarm_config = {
        .flags.auto_reload_on_alarm = 1,
		.alarm_count = tf,
    };
	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_speed, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer_speed));
  }
  else
  {
    ESP_ERROR_CHECK(gptimer_stop(gptimer_speed));
  }

}

void app_main(void)
{
	char str_output[17];
	SSD1306_t dev;

	setup();

#if CONFIG_I2C_INTERFACE
	ESP_LOGI(tag, "INTERFACE is i2c");
	ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_SPI_INTERFACE
	ESP_LOGI(tag, "INTERFACE is SPI");
	ESP_LOGI(tag, "CONFIG_MOSI_GPIO=%d",CONFIG_MOSI_GPIO);
	ESP_LOGI(tag, "CONFIG_SCLK_GPIO=%d",CONFIG_SCLK_GPIO);
	ESP_LOGI(tag, "CONFIG_CS_GPIO=%d",CONFIG_CS_GPIO);
	ESP_LOGI(tag, "CONFIG_DC_GPIO=%d",CONFIG_DC_GPIO);
	ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_SPI_INTERFACE

#if CONFIG_FLIP
	dev._flip = true;
	ESP_LOGW(tag, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
	ESP_LOGI(tag, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
	ESP_LOGI(tag, "Panel is 128x32");
	ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

#if CONFIG_SSD1306_128x64

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, "Speed [km/h]", 12, false);
    ssd1306_display_text(&dev, 4, "Engine [rpm]", 17, false);


while(1)
{
	if(mode != old_mode)
	{
		ssd1306_clear_line(&dev,2,false);
		ssd1306_clear_line(&dev,6,false);
		if(mode == SPEED_MODE)
		{
			sprintf(str_output,">%15d",vehiclespeed);	
			ssd1306_display_text(&dev,2,str_output, 16, false);
			sprintf(str_output,"%16d",enginespeed);
   		    ssd1306_display_text(&dev,6,str_output, 16, false);

		}
		else
		{
			sprintf(str_output,">%15d",enginespeed);
			ssd1306_display_text(&dev,6,str_output, 16, false);
			sprintf(str_output,"%16d",vehiclespeed);
			ssd1306_display_text(&dev,2,str_output, 16, false);	
		}
		old_mode = mode;
	}	
	if(vehiclespeed != old_vehiclespeed)
	{
		calcSpeedFrequency();
		ssd1306_clear_line(&dev,2,false);
		if(mode == SPEED_MODE)
			sprintf(str_output,">%15d",vehiclespeed);
		else
			sprintf(str_output,"%16d",vehiclespeed);
		ssd1306_display_text(&dev,2,str_output, 16, false);
		old_vehiclespeed = vehiclespeed;
	}
	if(enginespeed != old_enginespeed)
	{
		calcRPMFrequency();
		ssd1306_clear_line(&dev,6,false);
		if(mode == RPM_MODE)
			sprintf(str_output,">%15d",enginespeed);
		else
			sprintf(str_output,"%16d",enginespeed);
		ssd1306_display_text(&dev,6,str_output, 16, false);
		old_enginespeed = enginespeed;
	}
#endif // CONFIG_SSD1306_128x64


#if CONFIG_SSD1306_128x32
	ESP_LOGI(tag, "Only panel with 128x64 supported");
#endif // CONFIG_SSD1306_128x32

}


	
	
#if 0
	// Fade Out
	for(int contrast=0xff;contrast>0;contrast=contrast-0x20) {
		ssd1306_contrast(&dev, contrast);
		vTaskDelay(40);
	}
#endif

	// Restart module
	esp_restart();
}