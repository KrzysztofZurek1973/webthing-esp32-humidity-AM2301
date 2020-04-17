#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "simple_web_thing_server.h"
#include "am2301_humidity_sensor.h"

//#define ESP_INTR_FLAG_DEFAULT	0
#define AM2301_DATA_GPIO		CONFIG_AM2301_GPIO
#define AM2301_PIN_MASK			(1ULL << AM2301_DATA_GPIO)
#define NR_OF_PROBES 			50
#define LAST_EDGE 				42
#define TIMER_IDX				TIMER_1
#define TIMER_GR				TIMER_GROUP_0
//constants for digital bessel filer (after www-users.cs.york.ac.uk)
#define NZEROS 9
#define NPOLES 9
#define GAIN   4.161383162e+07

static xSemaphoreHandle DRAM_ATTR sem_data = NULL;
xSemaphoreHandle DRAM_ATTR am2301_mux;

uint64_t DRAM_ATTR data_time_tab[NR_OF_PROBES]; //time probes
static int32_t DRAM_ATTR edge_nr = 0;
static int32_t reading_step = 0;
gpio_config_t io_conf_input, io_conf_output;
//unsigned int us = 0;
static double temperature = 0.0, humidity = 0.0, dew_point = 0.0;

thing_t *thing_am2301 = NULL;
property_t *prop_humidity, *prop_temperature, *prop_dew_point;
at_type_t humidity_prop_type, temperature_prop_type, dew_point_prop_type;
at_type_t thing_am2301_type;
static void us_timer_init(void);
//double bessel_filter(double input);
double bessel_filter(double input, double *xv, double *yv);

/************************************************
*
* AM2301 data interrupt
*
************************************************/
static void IRAM_ATTR am2301_ISR(void *arg){
    static uint64_t timer_counter_value;
    static portBASE_TYPE xHigherPriorityTaskWoken;

	TIMERG0.hw_timer[TIMER_IDX].update = 1;
	timer_counter_value = 
		((uint64_t) TIMERG0.hw_timer[TIMER_IDX].cnt_high) << 32
	       			| TIMERG0.hw_timer[TIMER_IDX].cnt_low;
	    
	data_time_tab[edge_nr++] = timer_counter_value;
	
	if (edge_nr == LAST_EDGE){
	   	gpio_intr_disable(AM2301_DATA_GPIO);
	    xSemaphoreGiveFromISR(sem_data, &xHigherPriorityTaskWoken);
	    if (xHigherPriorityTaskWoken == true){
			portYIELD_FROM_ISR();
		}
	}
}


/*****************************************************
*
* dew point calculation, code downloaded from:
* https://www.best-microcontroller-projects.com/dht22.html
*
******************************************************/
double dew_point_calc(double celsius, double humidity){

  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

  // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;

  // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP / 0.61078); // temp var
  return (241.88 * T) / (17.558 - T);
}


//*********************************************************
//humidity reader thread
void am2301_humidity_reader_fun(void *pvParameter){
    uint64_t dt;
    uint64_t temp = 0;
	int all_probes = 0, ok = 0;
	double prev_temp = 0.0, prev_humi = 0.0, prev_dew = 0.0;
	time_t time_now, prev_time_temp, prev_time_humi, prev_time_dew;

#ifdef CONFIG_ENABLE_DIGITAL_FILTER
	bool first_probe = true;
	double xv_temp[NZEROS+1], yv_temp[NPOLES+1]; //temperature filter vectors
	double xv_humi[NZEROS+1], yv_humi[NPOLES+1]; //humidity filter vectors
#endif

    printf("humidity reader started\n");
    time(&time_now);
	prev_time_temp = time_now;
	prev_time_humi = time_now;
	prev_time_dew = time_now;
	
    while(1){
        xSemaphoreTake(sem_data, portMAX_DELAY);
        switch(reading_step){
            case 0:
                //start measuring procedure (see AM2301 datasheet)
                
                gpio_intr_disable(AM2301_DATA_GPIO);
                gpio_config(&io_conf_output);
                gpio_set_level(AM2301_DATA_GPIO, 0);
                
                vTaskDelay(10 / portTICK_RATE_MS); //set data bus LOW for 10 ms
				
				//gpio_set_level(AM2301_DATA_GPIO, 1);
				edge_nr = 0;
				gpio_intr_enable(AM2301_DATA_GPIO);
				gpio_config(&io_conf_input);
                break;
            
            case 1:
            	//all data received
                temp = 0ULL;
				
                for (int i = 2; i < LAST_EDGE; i++){
                    dt = data_time_tab[i] - data_time_tab[i - 1];
                    if (dt > 110){
                        temp = temp | 1ULL;
                    }
                    else{
                        temp = temp & (~1ULL);
                    }

                    temp = temp << 1;
                }
                uint8_t parity, parity_calc;

                //check parity byte
                temp = temp >> 1;
                parity = temp;
                parity_calc = (temp >> 8) + (temp >> 16) + (temp >> 24) +
                                (temp >> 32);
				all_probes++;
                if (parity == parity_calc){
                	int32_t t1, h1;
                	double t2, h2;
                	
					ok++;
                    t1 = temp >> 8;
                	h1 = temp >> 24;
					//check if temperature is below 0 Celcius
					if ((t1 & 0x00008000) == 0){
	                	t1 = t1 & 0x0000FFFF;
					}
					else{
						t1 = -(t1 & 0x00007FFF);
					}
	                t2 = (double)t1/10; //raw temperature
	                h1 = h1 & 0x00000FFF;
	                h2 = (double)h1/10; //raw humidity
	                
#ifdef CONFIG_ENABLE_DIGITAL_FILTER
	                if (first_probe == true){
	                	//initialize digital filter
	                	first_probe = false;
	                	for (int i = 0; i <= NZEROS; i++){
							xv_temp[i] = t2 / GAIN;
							yv_temp[i] = t2;
							xv_humi[i] = h2 / GAIN;
							yv_humi[i] = h2;
						}
					}
					//AM2301 produces spikes in temperature data
					//digital filter shoud remove them
	                t2 = bessel_filter(t2, xv_temp, yv_temp);
	                h2 = bessel_filter(h2, xv_humi, yv_humi);
#endif	                
	                
	                xSemaphoreTake(am2301_mux, portMAX_DELAY);
	                temperature = t2;
	                humidity = h2;
	                xSemaphoreGive(am2301_mux);
	                
	                dew_point = dew_point_calc(temperature, humidity);
	                
	                //inform subscribers
	                time(&time_now);
					//temperature
					double dt = fabs(temperature - prev_temp);
					if ((dt >= 0.1) || ((time_now - prev_time_temp) >= 30)){
						int8_t s = inform_all_subscribers_prop(prop_temperature);
						if (s == 0){
							prev_temp = temperature;
							prev_time_temp = time_now;
						}
						else{
							printf("temperature NOT sent\n");
						}
					}
					//humidity
					double dh = fabs(humidity - prev_humi);
					if ((dh >= 1.0) || ((time_now - prev_time_humi) >= 30)){
						int8_t s = inform_all_subscribers_prop(prop_humidity);
						if (s == 0){
							prev_humi = humidity;
							prev_time_humi = time_now;
						}
						else{
							printf("humidity NOT sent, dt: %i\n", (int32_t)(time_now - prev_time_temp));
						}
					}
					//dew point
					double ddp = fabs(dew_point - prev_dew);
					if ((ddp >= 0.5) || ((time_now - prev_time_dew) >= 30)){
						int8_t s = inform_all_subscribers_prop(prop_dew_point);
						if (s == 0){
							prev_dew = dew_point;
							prev_time_dew = time_now;
						}
						else{
							printf("dew point NOT sent, dt: %i\n", (int32_t)(time_now - prev_time_temp));
						}
					}
					printf("| OK-%i-%i | T: %4.3f; H: %4.1f; D: %3.1f\n",
						all_probes, ok, temperature, humidity, dew_point);
                }
                break;
                
            default:
                printf("reading_step ERROR!\n");
        }
        reading_step++;
    }
}


//*********************************************************
//humidity timer thread
void humidity_timer_fun(void *pvParameter){
	//int cnt = 0;
	vTaskDelay(5000 / portTICK_RATE_MS);
	
    while(1) {
        vTaskDelay(5000 / portTICK_RATE_MS);
        //printf("main task: %d\n", cnt++);
        reading_step = 0;
        //for (int i = 0; i < NR_OF_PROBES; i++){
        //   data_time_tab[i] = 0;
        //}
        xSemaphoreGive(sem_data);
    }
}


/***************************************************
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 ***************************************************/
static void us_timer_init(){
	timer_config_t config;
	
    /* Select and initialize basic parameters of the timer */
    config.divider = 80; //measure micro seconds
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = false;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TIMER_AUTORELOAD_DIS;
    
    timer_init(TIMER_GR, TIMER_IDX, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GR, TIMER_IDX, 0x00000000ULL);
    timer_start(TIMER_GR, TIMER_IDX);
}


/*****************************************************************
 *
 * Initialize button thing and all it's properties and event
 *
 * ****************************************************************/
thing_t *init_humidity_sensor_am2301(char *_thing_id){

	//input configuration
    io_conf_input.intr_type = GPIO_INTR_NEGEDGE; //interrupt of falling edge
    io_conf_input.pin_bit_mask = AM2301_PIN_MASK; //bit mask of the pins
    io_conf_input.mode = GPIO_MODE_INPUT;
    io_conf_input.pull_up_en = 1;
    io_conf_input.pull_down_en = 0;
    //output configuration
    io_conf_output.intr_type = GPIO_INTR_NEGEDGE; //interrupt of falling edge
    io_conf_output.pin_bit_mask = AM2301_PIN_MASK; //bit mask of the pins
    io_conf_output.mode = GPIO_MODE_OUTPUT;
    io_conf_output.pull_up_en = 0;
    io_conf_output.pull_down_en = 0;
    
    //at start set output mode
    gpio_config(&io_conf_output);
    gpio_intr_disable(AM2301_DATA_GPIO);
    gpio_set_level(AM2301_DATA_GPIO, 1);
	
	//create semaphore 
    sem_data = xSemaphoreCreateBinary();
    //create mutex
	am2301_mux = xSemaphoreCreateMutex();
	
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(AM2301_DATA_GPIO, am2301_ISR, NULL);
    
    us_timer_init();
    
    //start timer task
    xTaskCreate(&humidity_timer_fun, "humidity timer", 1024, NULL, 1, NULL);
    //start humidity reader task
    xTaskCreate(&am2301_humidity_reader_fun, "humidity reader", 2048*2, NULL, 2, NULL);
    
    //create humidity sensor thing
	thing_am2301 = thing_init();
    
    thing_am2301 -> id = _thing_id;
	thing_am2301 -> at_context = things_context;
	thing_am2301 -> model_len = 1500;
	//set @type
	thing_am2301_type.at_type = "MultiLevelSensor";
	thing_am2301_type.next = NULL;
	set_thing_type(thing_am2301, &thing_am2301_type);
	thing_am2301 -> description = "Internet connected humidity sensor AM2301";
	
	//create humidity property
	prop_humidity = property_init(NULL, NULL);
	prop_humidity -> id = "humidity";
	prop_humidity -> description = "relative humidity";
	humidity_prop_type.at_type = "LevelProperty";
	humidity_prop_type.next = NULL;
	prop_humidity -> at_type = &humidity_prop_type;
	prop_humidity -> type = VAL_NUMBER;
	prop_humidity -> value = &humidity;
	prop_humidity -> max_value.float_val = 100;
	prop_humidity -> min_value.float_val = 0;
	prop_humidity -> unit = "percent";
	prop_humidity -> title = "Humidity";
	prop_humidity -> read_only = true;
	prop_humidity -> set = NULL;
	prop_humidity -> mux = am2301_mux;

	add_property(thing_am2301, prop_humidity); //add property to thing
	
	//create temperature property
	prop_temperature = property_init(NULL, NULL);
	prop_temperature -> id = "temperature";
	prop_temperature -> description = "temperature";
	temperature_prop_type.at_type = "TemperatureProperty";
	temperature_prop_type.next = NULL;
	prop_temperature -> at_type = &temperature_prop_type;
	prop_temperature -> type = VAL_NUMBER;
	prop_temperature -> value = &temperature;
	prop_temperature -> max_value.float_val = 80;
	prop_temperature -> min_value.float_val = -40;
	prop_temperature -> unit = "degree celsius";
	prop_temperature -> title = "Temperature";
	prop_temperature -> read_only = true;
	prop_temperature -> set = NULL;
	prop_temperature -> mux = am2301_mux;

	add_property(thing_am2301, prop_temperature); //add property to thing
	
	//create dew point property
	prop_dew_point = property_init(NULL, NULL);
	prop_dew_point -> id = "dew_point";
	prop_dew_point -> description = "dew point";
	dew_point_prop_type.at_type = "TemperatureProperty";
	dew_point_prop_type.next = NULL;
	prop_dew_point -> at_type = &dew_point_prop_type;
	prop_dew_point -> type = VAL_NUMBER;
	prop_dew_point -> value = &dew_point;
	prop_dew_point -> max_value.float_val = 40;
	prop_dew_point -> min_value.float_val = -20;
	prop_dew_point -> unit = "degree celsius";
	prop_dew_point -> title = "Dew point";
	prop_dew_point -> read_only = true;
	prop_dew_point -> set = NULL;
	prop_dew_point -> mux = am2301_mux;

	add_property(thing_am2301, prop_dew_point); //add property to thing
	
	return thing_am2301;
}

#ifdef CONFIG_ENABLE_DIGITAL_FILTER
/**************************************************
*
* Digital IIR Bessel filter
* filter parameters calculated by tool on www-users.cs.york.ac.uk/~fisher/mkfilter/
* for the following input parameters:
*	- filter type: Bessel
*	- filter order: 9
*	- sample rate: 0.2 [Hz]
*	- corner frequency: 0.005 [Hz]
* other parameters left blank
*
***************************************************/
double bessel_filter(double input, double *xv, double *yv){
	xv[0] = xv[1];
	xv[1] = xv[2];
	xv[2] = xv[3];
	xv[3] = xv[4];
	xv[4] = xv[5];
	xv[5] = xv[6];
	xv[6] = xv[7];
	xv[7] = xv[8];
	xv[8] = xv[9];
	xv[9] = input / GAIN;

	yv[0] = yv[1];
	yv[1] = yv[2];
	yv[2] = yv[3];
	yv[3] = yv[4];
	yv[4] = yv[5];
	yv[5] = yv[6];
	yv[6] = yv[7];
	yv[7] = yv[8];
	yv[8] = yv[9];
	yv[9] = (xv[0] + xv[9]) + 9 * (xv[1] + xv[8]) + 36 * (xv[2] + xv[7])
	         + 84 * (xv[3] + xv[6]) + 126 * (xv[4] + xv[5])
	         + (  0.1248557410 * yv[0]) + ( -1.3873314615 * yv[1])
	         + (  6.8825975493 * yv[2]) + (-20.0130813270 * yv[3])
	         + ( 37.5974890690 * yv[4]) + (-47.3352407920 * yv[5])
	         + ( 39.9486773120 * yv[6]) + (-21.7990708280 * yv[7])
	         + (  6.9810924340 * yv[8]);
	return yv[9];
}
#endif

