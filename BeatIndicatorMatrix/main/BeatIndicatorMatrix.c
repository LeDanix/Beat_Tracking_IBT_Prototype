/* Beat Indicator

   This code is made by Daniel Saiz Azor, a student at the
   University of Zaragoza.
   It is a part of his TFG where he implements the IBT beat detection
   software functionality in the ESP-32 microcontroller.

   04/05/2021 
*/


/* Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_system.h"


/* Defines */
//SPI
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define INTENSITY_LEVEL 1 //Value between [0-15]
#define MAXCOLUMNS 8

//MATRIX
#define OP_DECODEMODE  0x09
#define OP_INTENSITY   0x0a
#define OP_SCANLIMIT   0x0b
#define OP_SHUTDOWN    0x0c
#define OP_DISPLAYTEST 0x0f

//PROGRAM
#define DEBUG 0
#define TWDT_TIMEOUT_S 4

//TIMER
#define TIMERG0 0
#define TIMER0 0

//GPIO
#define GPIO_BIT_MASK (1ULL<<GPIO_NUM_34)

//UART
#define COUNTER_DIV 32  /* Max divider */
#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

/* Function delcarations */
void make_transaction(uint8_t* data, size_t dataLen);
void spi_transfer(volatile uint8_t opcode, volatile uint8_t data);
void shutdown_spi(char b);
void clear_display(void);
void draw_matrix(uint8_t matrix[MAXCOLUMNS]);
void spi_init(void);


/* Global variables declaration */
uint8_t colection_matrix[37][MAXCOLUMNS] = {
								   			{0x18, 0x3c, 0x7e, 0xff, 0xff, 0x7e, 0x3c, 0x18}, //Diamond
											{0xff, 0x81, 0xbd, 0xa5, 0xa5, 0xbd, 0x81, 0xff}, //Square
											{0x18, 0xa5, 0xbd, 0xe7, 0xe7, 0xbd, 0xa5, 0x18}, //Random1
											{0x18, 0x18, 0x24, 0xc3, 0xc3, 0x24, 0x18, 0x18}, //Diamond2
											{0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81}, //X
											{0x12, 0x24, 0x49, 0x92, 0x92, 0x49, 0x24, 0x12}, //Flechas1
											{0x48, 0x24, 0x92, 0x49, 0x49, 0x92, 0x24, 0x48}, //Flechas2
											{0x4e, 0x11, 0x91, 0x11, 0x6e, 0x90, 0x91, 0x64}, //Bubbles1
											{0x00, 0x00, 0x20, 0x1f, 0x10, 0x14, 0x12, 0x10}, //StepPokemon
											{0x11, 0x12, 0x14, 0xf0, 0x0f, 0x28, 0x48, 0x88}, //Random2
											{0xa1, 0x46, 0x92, 0x20, 0x04, 0x49, 0x62, 0x85}, //Random3
											{0xa5, 0x5a, 0xa5, 0x42, 0x42, 0xa5, 0x5a, 0xa5}, //Random4
											{0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0}, //Rayas
											{0x7e, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x7e}, //I
											{0x7f, 0xc6, 0xc6, 0xc6, 0x7e, 0xc6, 0xc6, 0x7f}, //B
											{0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x99, 0xff}, //T
											{0x36, 0x14, 0x1b, 0x3c, 0xd8, 0x24, 0x24, 0x3c}, //Boy
											{0xfe, 0xe6, 0xe9, 0xe0, 0x00, 0x00, 0x00, 0x00}, //snail
											{0x09, 0x06, 0x0e, 0x1d, 0x38, 0x70, 0x60, 0x80}, //Sword
											{0xff, 0x7e, 0x3c, 0x18, 0x18, 0x3c, 0x7e, 0xff}, //Sand-clock
											{0x00, 0x7c, 0x78, 0x70, 0x68, 0x44, 0x02, 0x01}, //Arrow
											{0x3c, 0x24, 0xe7, 0x99, 0x99, 0xe7, 0x24, 0x3c}, //Flower
											{0x99, 0x66, 0x24, 0x24, 0x5a, 0xa5, 0x5a, 0x81},
                                            {0x7e, 0x7e, 0x00, 0x3c, 0x3c, 0x00, 0x7e, 0x7e},
                                            {0x18, 0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x66},
                                            {0x7e, 0x81, 0xb9, 0x91, 0xa5, 0x81, 0x5a, 0x24},
                                            {0x99, 0x99, 0x5a, 0x3c, 0x18, 0xff, 0xc3, 0xdb},
                                            {0x00, 0x18, 0x3c, 0x7e, 0xff, 0x7e, 0x3c, 0x00},
                                            {0x00, 0x00, 0x3c, 0x42, 0xff, 0x5a, 0x24, 0x00},
                                            {0x3e, 0x3e, 0x2a, 0x3e, 0x3e, 0x63, 0x63, 0x63},
                                            {0x66, 0x7e, 0x3c, 0x7e, 0xe7, 0xdb, 0x24, 0x00},
                                            {0x56, 0x31, 0x31, 0x56, 0x21, 0x53, 0x51, 0x57},
                                            {0x18, 0xbd, 0x7e, 0x66, 0xbd, 0x18, 0x18, 0x24}, //LadyBug
                                            {0xf8, 0x18, 0x18, 0x28, 0x4f, 0x89, 0x99, 0x69}, //42
                                            {0x00, 0x3c, 0x24, 0x3c, 0x7e, 0x5a, 0x7e, 0x3c}, //Crain
                                            {0x3e, 0x24, 0x28, 0x3c, 0x22, 0x4a, 0x34, 0x08}, //42 Simbol
                                            {0x18, 0x3c, 0x5a, 0x5a, 0xff, 0xe7, 0xc3, 0xc3}, //Bat
                                            {0x81, 0x66, 0x18, 0x18, 0x18, 0x99, 0x66, 0x18}  //Boy2
                                            };

uint8_t full_matrix[MAXCOLUMNS] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const char* LOG_TAG = "SPI";
static const char *TAG = "uart_events";
double time1;
uint8_t ACT_LED;

//SPI Handler
spi_device_handle_t spi;
//UART Handler
static QueueHandle_t uart0_queue;

/* UART Task */
static void uart_event_task(void *pvParameters)
{
	/* UART Task 
		Control the serial data which provide 
		the serial port COM. 
		ACT_LED only set to 1 when UART data is '1'
	*/
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE); //Allocates 1024 positions of memory
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) { //Wait for a event on the queue
            bzero(dtmp, RD_BUF_SIZE); //Delete
            #if DEBUG
                ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            #endif
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    #if DEBUG
                        ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                        ESP_LOGI(TAG, "[DATA EVT]:");
                    #endif
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);  //Read events
                    //uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size); //Echo
                    switch((int)(dtmp[0]-'0')){ //Treat event
                        case 1:
                            ACT_LED = 1;
                            break;
                    }
                    break;

                //Event FIFO overflow detected
                case UART_FIFO_OVF:
                    #if DEBUG
                        ESP_LOGI(TAG, "hw fifo overflow");
                    #endif
                    // If fifo overflow happened, directly flush the rx buffer here 
                    // in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                //Event UART ring buffer full
                case UART_BUFFER_FULL:
                    #if DEBUG
                        ESP_LOGI(TAG, "ring buffer full");
                    #endif
                    // If buffer full happened, directly flush the rx buffer here 
                    // in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                //Event UART RX break detected
                case UART_BREAK:
                    #if DEBUG
                        ESP_LOGI(TAG, "uart rx break");
                    #endif
                    break;

                //Event UART parity check error
                case UART_PARITY_ERR:
                    #if DEBUG
                        ESP_LOGI(TAG, "uart parity error");
                    #endif
                    break;

                //Event UART frame error
                case UART_FRAME_ERR:
                    #if DEBUG
                        ESP_LOGI(TAG, "uart frame error");
                    #endif
                    break;

                //UART_PATTERN_DET
                /* case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size); //Get ring buffer data length
                    int pos = uart_pattern_pop_pos(EX_UART_NUM); //Return first pattern position
                    #if DEBUG
                        ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    #endif
                    if (pos == -1) {
                        // Not exists pattern position, directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    }else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS); //Read first position pattern
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat)); //Put 0 into the 4 first position of pat array
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS); //Read data and put into pat array
                        #if DEBUG
                            ESP_LOGI(TAG, "read data: %s", dtmp);
                            ESP_LOGI(TAG, "read pat : %s", pat);
                        #endif
                    }
                    break; */
                //Others
                default:
                    #if DEBUG
                        ESP_LOGI(TAG, "uart event type: %d", event.type);
                    #endif
                    break;
            }
        }
        vTaskDelay(10); //Avoid problems
    }
    free(dtmp); //Release reserved memory locations
    dtmp = NULL;
    vTaskDelete(NULL);
}


static void beat_light(void *pvParameters){
	/* Beat Task 
		Control the LED matrix 8x8 IN 2 MODES
		INPUT GPIO_34 == 1: Normal mode, matrix turn on 64 LEDS
		INPUT GPIO_34 == 0: Rave mode, matrix turn on some draws
		*/
	uint8_t n = 0;
    for(;;){
        int light = gpio_get_level(GPIO_NUM_34);
        #if DEBUG
            if (light == 1) printf("No pulsado \n"); else printf("Pulsado \n");
        #endif
		//Turn on matrix
		if(!gpio_get_level(GPIO_NUM_34) && ACT_LED){
			timer_set_counter_value(TIMERG0, TIMER0, 0);

			// Draw in order 
            //if (n != sizeof(colection_matrix)/8 - 1) n++; else n=0;
			//draw_matrix(colection_matrix[n]);

            //Draw aleatory
            n = (rand() % ((sizeof(colection_matrix)/8) + 1));
            draw_matrix(colection_matrix[n]);

			vTaskDelay(10/ portTICK_PERIOD_MS);
		}else if (gpio_get_level(GPIO_NUM_34) && ACT_LED){
			timer_set_counter_value(TIMERG0, TIMER0, 0);
			draw_matrix(full_matrix);
			vTaskDelay(10/ portTICK_PERIOD_MS);
		}
		ACT_LED = 0;
        timer_get_counter_time_sec(TIMERG0, TIMER0, &time1);
        if(time1 > 0.20){ //Wait for 0.15s to set LOW LEDs
			clear_display();
			vTaskDelay(10/ portTICK_PERIOD_MS);
            //To avoid counter keeps permanently increasing 
            //timer_set_counter_value(TIMERG0, TIMER0, 281250); //281250 == 0.15s with a DIV of 32
            timer_set_counter_value(TIMERG0, TIMER0, 374994); // 0.15s
        }
        vTaskDelay(10);
    }
    vTaskDelete(NULL);
}


void make_transaction(uint8_t* data, size_t dataLen) {
	/* Configure data to transmit into SPI */
	assert(data != NULL);
	assert(dataLen > 0);
	spi_transaction_t trans_desc = {
		.flags     = 0,
		.length    = dataLen * 8,
		.tx_buffer = data,
	};
	spi_device_transmit(spi, &trans_desc);
}

void spi_transfer(volatile uint8_t opcode, volatile uint8_t data) {
	/* Join command data and LEDS data to send into SPI */
	uint8_t spidata[2];
	for (int i = 0; i < 2; i++) {
		spidata[i] = (uint8_t) 0;
	}
	spidata[0] = opcode;
	spidata[1] = data;
	make_transaction(spidata, 2);
}

void clear_display(void){
	/* Clear 8x8 display */
	for(char i = 1; i < MAXCOLUMNS + 1; i++){
		spi_transfer(i, 0x00);
	}
}

void shutdown_spi(char b){
	/* Shutdown or Turn on the device */
	if (b) spi_transfer(OP_SHUTDOWN, 0);
	else spi_transfer(OP_SHUTDOWN, 1);
}

void draw_matrix(uint8_t matrix[MAXCOLUMNS]) {
	/* Draw a especific picture into matrix */
	for(uint8_t idx_mtx = 1; idx_mtx < MAXCOLUMNS+1; idx_mtx++){
		spi_transfer(idx_mtx, matrix[idx_mtx-1]);
		vTaskDelay(5/ portTICK_PERIOD_MS);
	}
}

void spi_init(void){
	/* Initialise the 8x8 matrix device */

	/* Probe the display */
	spi_transfer(OP_DISPLAYTEST, 1);
	vTaskDelay(50/ portTICK_PERIOD_MS);

	spi_transfer(OP_DISPLAYTEST, 0);
	vTaskDelay(50/ portTICK_PERIOD_MS);

	/* Set the limit */
	spi_transfer(OP_SCANLIMIT, 7);
	vTaskDelay(50/ portTICK_PERIOD_MS);

	/* Set mode 0x00 */
	spi_transfer(OP_DECODEMODE, 0);
	vTaskDelay(50/ portTICK_PERIOD_MS);

	/* Set intensity level */
	spi_transfer(OP_INTENSITY, INTENSITY_LEVEL);
	vTaskDelay(50/ portTICK_PERIOD_MS);

	/* Clear display */
	clear_display();
	vTaskDelay(50/ portTICK_PERIOD_MS);

	/* Turn on */
	shutdown_spi(0);
	vTaskDelay(50/ portTICK_PERIOD_MS);
}

void app_main(void)
{	
	esp_err_t ret;

	#if DEBUG
        esp_log_level_set(TAG, ESP_LOG_INFO);
    #endif
    //esp_task_wdt_init(TWDT_TIMEOUT_S, true);
    esp_task_wdt_deinit();

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    /* Configure parameters of GPIOs */
    gpio_config_t gpio_configu = {
        .pin_bit_mask = GPIO_BIT_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    /* Configure parameters of Timer 0 Group 0 */
    timer_config_t timer_configu = {
        .alarm_en = TIMER_ALARM_DIS,
        .counter_en = TIMER_START,
        .counter_dir = TIMER_COUNT_MAX,
        .divider = COUNTER_DIV,
    };

	/* Configure SPI bus */
	spi_bus_config_t spi_config_bus = {
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
		.max_transfer_sz = 0,
		//.flags = (SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI)
	};

	/* Configure SPI interface */
	spi_device_interface_config_t spi_config_interface = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 0,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = 10*1000*1000,
		.spics_io_num = PIN_NUM_CS,
		.flags = SPI_DEVICE_NO_DUMMY,
		.queue_size = 1,
		.pre_cb = NULL,
		.post_cb  = NULL,
	};

	//Config GPIO 33 as OUTPUT
    gpio_config(&gpio_configu);

    //Config Timer
    timer_init(TIMERG0, TIMER0, &timer_configu);

    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    #if DEBUG
        //Set UART log level
        esp_log_level_set(TAG, ESP_LOG_INFO);
    #endif
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    //uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    //uart_pattern_queue_reset(EX_UART_NUM, 20);

	//Initialize SPI bus
	ret = spi_bus_initialize(SPI3_HOST, &spi_config_bus, 1);
	#if DEBUG
		if (ret != ESP_OK) ESP_LOGE(LOG_TAG, "spi_bus_initialize: %d", ret);
	#endif

	//Initialize SPI bus Device
	ret = spi_bus_add_device(SPI3_HOST, &spi_config_interface, &spi);
	#if DEBUG
		if (ret != ESP_OK) ESP_LOGE(LOG_TAG, "spi_bus_add_device: %d", ret);
	#endif

	spi_init();


	//xTaskCreate(set_matrix_task, "setMatrix", 1024, NULL, 12, NULL);
	//xTaskCreate(probe, "probe", 1024, NULL, 12, NULL);
	xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(beat_light, "beat_light", 2048, NULL, 1, NULL); 
}