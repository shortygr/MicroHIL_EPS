
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

#define tag "SSD1306"

#define ENCODER_PIN_A 26
#define ENCODER_PIN_B 27
#define BUTTON_PIN 25
#define GPIO_BIT_MASK_ENCODER  ((1ULL<<GPIO_NUM_26) | (1ULL<<GPIO_NUM_27)) 
#define GPIO_BIT_MASK_BUTTON  (1ULL<<GPIO_NUM_25)


#define SPEED_MODE 0
#define RPM_MODE   1

#define LOW 0
#define HIGH 1

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
int debounce_time_encoder = 10;
int incSpeed = 5;
int incRPM = 500;
int maxSpeed = 300;
int maxRPM = 16000;
int encoderPos = 0;



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

void setup(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_BIT_MASK_ENCODER;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
 	io_conf.pin_bit_mask = GPIO_BIT_MASK_BUTTON; 
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
	gpio_isr_handler_add(ENCODER_PIN_A, encoder_interrupt_handler, (void*)ENCODER_PIN_A);
	gpio_isr_handler_add(ENCODER_PIN_B, encoder_interrupt_handler, (void*)ENCODER_PIN_B);
	gpio_isr_handler_add(BUTTON_PIN, button_interrupt_handler, (void*)BUTTON_PIN);

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