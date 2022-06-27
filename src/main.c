#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "c_timeutils.h"
#include "driver/adc.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include <hd44780.h>
#include <pcf8574.h>
#include <i2cdev.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) do{ esp_err_t rc = (x); if(rc != ESP_OK){ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} }while(0);

//°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°[ CPU #1 ]°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
//--------------------------------------------------------------------------------------------------------------------------------------------BLINK TASK
bool led_GUI = false;
//--------------------------------------------------------------------------------------------------------------------------------------------ENCODER AND CONTROL
#define RE_A_GPIO   14
#define RE_B_GPIO   27
#define RE_BTN_GPIO 26
#define BTN_CONTROL_ON_OFF 25
static const int8_t valid_states[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
int8_t inc;
float inc_f;
uint8_t code;
uint16_t store;
bool ok_set_parameters = false;
bool set_mode_ToM = true;//true->Set Temperature | false->Set Motor Velocity
//°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°[ CPU #0 ]°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
//--------------------------------------------------------------------------------------------------------------------------------------------WIFI & UDP
//########## ACTIVATE WIFI AND UDP ##########
bool wifi_udp_activation = true;

//########## SELECT WIFI MODE [AP or STA] ##########
#define AP "Wifi_Mode"

//-----STA-----[NOTA: EL ESP32 SE CONECTA AL MODEM]
#define WIFI_SSID "TP-Link_AD08"
#define WIFI_PASS "password"
ip_event_got_ip_t* event_got_ip;
char my_ip[32];

//-----AP-----[NOTA: EL ESP32 FUNCIONA COMO MODEM]
#define WIFI_AP_SSID "ESP32"
#define WIFI_AP_PASS "youtube_2022"
wifi_event_ap_staconnected_t* event_ap_staconnected;
//ap_ip: 192.168.4.1

//-----UDP-----[NOTA: CONFIGURAR LA DIRECCION IP DEL DISPOSITIVO AL QUE SE LE ENVIA INFORMACION]
#ifdef STA
	#define HOST_IP_ADDR "192.168.2.101"//COMPRUEBE LA DIRECCION DEL DISPOSITIVO ASIGNADA POR EL MODEM
	#define WIFI_MODE 1//STA
#endif
#ifdef AP
	#define HOST_IP_ADDR "192.168.4.2"//DIRECCION ASIGNADA POR EL ESP32 AL PRIMER DISPOSITIVO EN CONECTARSE
	#define WIFI_MODE 2//AP
#endif
#define PORT 8080

int sock;
//Send
int send_flag;
char addr_str[32];
struct sockaddr_in dest_addr;
char buffer_send[32];
//Received
int recv_flag;
struct sockaddr_in source_addr;
socklen_t socklen = sizeof(source_addr);
char buffer_recv[8];

bool wifi_OK = false;
float flow_time_UDP = 0.0;
//--------------------------------------------------------------------------------------------------------------------------------------------GPIO
#define LED_A 2
//--------------------------------------------------------------------------------------------------------------------------------------------ADC
#define DEFAULT_VREF  1100
#define NO_OF_SAMPLES 64

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;//GPIO34 if ADC1
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

uint32_t adc_raw = 0;
uint32_t voltage = 0;

const float vcc = 3040.0;
const float res = 5520.0;
float rt = 0.0;

float loga;
const float aSH = 0.5543366916E-3; //¡¡¡Datos calculados para un caso concreto...
const float bSH = 2.305108199E-4;  //...debes cambiarlos por los que calcules tu...
const float cSH = 1.115502451E-7;  //...de tu propio termistor NTC!!!	

float invT = 0.0;
float temp = 0.0;

//Filter
float temp_x1 = 0.0;
const float kT = 0.2;
float voltage_x1 = 0.0;
const float kV = 0.2;
//--------------------------------------------------------------------------------------------------------------------------------------------PWM
/*
 * GPIO 13/12 are from high speed channel group.
 */
#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE                 
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1
#define PWM_CH0 (13) 
#define PWM_CH1 (12)

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_CHANNEL LEDC_CHANNEL_0
#define MOTOR_CH0 (15)                 


ledc_channel_config_t ledc_channel[3] = {
	{
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = PWM_CH0,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
    {
        .channel    = LEDC_HS_CH1_CHANNEL,
        .duty       = 0,
        .gpio_num   = PWM_CH1,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
	{
        .channel    = LEDC_LS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_CH0,
        .speed_mode = LEDC_LS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_LS_TIMER
    },
};

/*
* Bits: 10 "1023", 11 "2047", 12 "4095", 13 "8191", 14 " 16383" or 15 "32767"
*/
#define LEDC_TIMER_BIT LEDC_TIMER_12_BIT
uint32_t bitDec = 4095;//?

uint32_t freqHS = 10;//100 ms con 6 ciclos de la onda sinusoidal 
float dFactorHS = 0.0;
uint32_t dutyHS;

uint32_t freqLS = 10000;
float dFactorLS = 0.0;
uint32_t dutyLS;
void set_PWM_Duty_LS(void);
//--------------------------------------------------------------------------------------------------------------------------------------------I2C
#define SDA_GPIO 21
#define SCL_GPIO 22
#define I2C_ADDR 0x27

static i2c_dev_t pcf8574;
hd44780_t lcd_monitor;
float flow_time_LCD = 0.0;
//--------------------------------------------------------------------------------------------------------------------------------------------GUI
char charIN;
int numIN;
//--------------------------------------------------------------------------------------------------------------------------------------------CONTROL
//Constants for the PID controller
float refTemp = 225.0;//225.0
const float kP = 0.018;//0.016
const float kI = 0.000014;//0.000014 
const float kD = 0.0;       
const float iLIMIT = 200000.0;//Si ref_Temp = 225°C y no hay cambio durante 15 min el iLIMIT se satura

float error = 6.0;
float pPart = 0.0;
float iPart = 0.0;
float dPart = 0.0;

float integratedError = 0.0; 
float lastError = 0.0;
float pidValue = 0.0;
float flow_time_PID = 0.0;
bool control_ON = false;
//--------------------------------------------------------------------------------------------------------------------------------------------TIME
struct timeval tvalBefore, tvalNow, tval_dt;
uint32_t timeUS = 0;
float dt = 0.0;//Seconds
float flow_time_UART = 0.0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void feedTheDog(void)
{
  	//Feed dog 0
  	TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;//Write enable
  	TIMERG0.wdt_feed = 1;                      //Feed dog
  	TIMERG0.wdt_wprotect = 0;                  //Write protect               
}

void get_dt(bool timeLoop)
{		
	gettimeofday(&tvalNow, NULL);
	tval_dt = timeval_sub(&tvalNow, &tvalBefore);
	timeUS = tval_dt.tv_usec;
	dt = timeUS/1000000.0f;
	tvalBefore = tvalNow;
	
	if(wifi_OK){flow_time_UDP += dt;} else{flow_time_UART += dt;}
	flow_time_PID += dt;
	flow_time_LCD += dt;
	if(timeLoop)
		printf("\rTIME LOOP: %.4f", dt);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup_UDP(void)
{
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_port = htons(PORT);
	inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
	
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock < 0)
    {
        printf("Socket call failed\n");
        exit(0);
    }
    printf("Socket created\n");
}

void msg_UDP(void)
{
	send_flag = sendto(sock, buffer_send, strlen(buffer_send), MSG_DONTWAIT, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

void recv_UDP(void)//TEST
{
	recv_flag = recvfrom(sock, buffer_recv, sizeof(buffer_recv) - 1, MSG_DONTWAIT, (struct sockaddr *)&source_addr, &socklen);
    if(recv_flag > 0)
    {
        buffer_recv[recv_flag] = 0;//Null-terminate whatever we received and treat like a string
		printf("\rReceived %d bytes from %s:\n", recv_flag, addr_str);
        printf("%s", buffer_recv);
    }		
}

static void event_handler_wifi(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if(WIFI_MODE == 1)//STA
  	{
    	switch(event_id)
    	{
      		case WIFI_EVENT_STA_START:
        		ESP_ERROR_CHECK(esp_wifi_connect());
        		printf("Connecting...\n");
        		break;
      		case IP_EVENT_STA_GOT_IP:
			  	event_got_ip = (ip_event_got_ip_t*) event_data;
        		printf("\nOur IP address is: " IPSTR "\n", IP2STR(&event_got_ip->ip_info.ip));
		    	sprintf(my_ip, IPSTR, IP2STR(&event_got_ip->ip_info.ip));//Save IP
				wifi_OK = true;
        		break;
      		case WIFI_EVENT_STA_DISCONNECTED:
				wifi_OK = false;
        		ESP_ERROR_CHECK(esp_wifi_connect());
        		printf("Reconnecting...\n");
        		break;
      		default:
        		printf("event_id: %d\n", event_id);
        		break;
    	}
  	}
  	else
    	if(WIFI_MODE == 2)//AP
    	{
      		switch(event_id)
      		{		
        		case WIFI_EVENT_AP_START:
          			printf("Waiting new connection....\n");
          			break;
        		case WIFI_EVENT_AP_STACONNECTED:
					event_ap_staconnected = (wifi_event_ap_staconnected_t*) event_data;
					printf("station "MACSTR" join, AID=%d\n", MAC2STR(event_ap_staconnected->mac), event_ap_staconnected->aid);
	    			wifi_OK = true;
          			break;
        		case WIFI_EVENT_AP_STADISCONNECTED:
					wifi_OK = false;
          			break;
        		default:
          			printf("event_id: %d\n", event_id);
          			break;
      		}
    	}
}

void setup_WIFI(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());	
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler_wifi, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler_wifi, NULL, NULL));

   	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
   	//ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));//WiFi at boot time

	if(WIFI_MODE == 1)
	{
		//STA
		esp_netif_create_default_wifi_sta();
   		wifi_config_t staConfig = {
      		.sta = {
         		.ssid = WIFI_SSID,
         		.password = WIFI_PASS
      		}
   		};
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &staConfig));
		printf("WIFI_MODE_STA\n");
	}
	else
		if(WIFI_MODE == 2)
		{
			//AP
			esp_netif_create_default_wifi_ap();
			wifi_config_t apConfig = {
   				.ap = {
      				.ssid = WIFI_AP_SSID,
      				.ssid_len = strlen(WIFI_AP_SSID),
      				.password = WIFI_AP_PASS,
      				.channel = 0,
      				.authmode = WIFI_AUTH_OPEN,
      				.ssid_hidden = 0,
      				.max_connection = 4,
      				.beacon_interval = 100
				}
			};
			ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apConfig));
			printf("WIFI_MODE_AP\n");
		}
}

void setup_WIFI_UDP(void)
{
	if(wifi_udp_activation)
	{
		setup_WIFI();
		ESP_ERROR_CHECK(esp_wifi_start());
		setup_UDP();	
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup_BLINK(void)
{
	//********* LED_A *********
	gpio_pad_select_gpio(LED_A);
    gpio_set_direction(LED_A, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_A, 0);
}

void blink(void)
{
	gpio_set_level(LED_A, 0);
	vTaskDelay(100/portTICK_PERIOD_MS);
	gpio_set_level(LED_A, 1);
	vTaskDelay(100/portTICK_PERIOD_MS);
	gpio_set_level(LED_A, 0);
	vTaskDelay(100/portTICK_PERIOD_MS);
	gpio_set_level(LED_A, 1);
	vTaskDelay(100/portTICK_PERIOD_MS);
	gpio_set_level(LED_A, 0);
	vTaskDelay(100/portTICK_PERIOD_MS);
	gpio_set_level(LED_A, 1);
	vTaskDelay(100/portTICK_PERIOD_MS);
	gpio_set_level(LED_A, 0);
}

void blink_task(void *pvParameter)
{
	while(1)
	{
		//printf("Core: %d\n", xPortGetCoreID());

		if(led_GUI)
		{
			gpio_set_level(LED_A, 0);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_A, 1);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_A, 0);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_A, 1);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_A, 0);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_A, 1);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_A, 0);
			led_GUI = false;
		}

		if(abs(error) < 5)
		{
			gpio_set_level(LED_A, 1);
		}
		else
		{
			gpio_set_level(LED_A, 0);
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void encoder_task(void *arg)
{
	//Setup GPIO
	gpio_pad_select_gpio(RE_A_GPIO);
    gpio_set_direction(RE_A_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(RE_A_GPIO, GPIO_PULLUP_ONLY); 
	gpio_set_intr_type(RE_A_GPIO, GPIO_INTR_DISABLE);	

	gpio_pad_select_gpio(RE_B_GPIO);
    gpio_set_direction(RE_B_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(RE_B_GPIO, GPIO_PULLUP_ONLY); 
	gpio_set_intr_type(RE_B_GPIO, GPIO_INTR_DISABLE);

	gpio_pad_select_gpio(RE_BTN_GPIO);
    gpio_set_direction(RE_BTN_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(RE_BTN_GPIO, GPIO_PULLUP_ONLY); 
	gpio_set_intr_type(RE_BTN_GPIO, GPIO_INTR_DISABLE);

	gpio_pad_select_gpio(BTN_CONTROL_ON_OFF);
    gpio_set_direction(BTN_CONTROL_ON_OFF, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BTN_CONTROL_ON_OFF, GPIO_PULLUP_ONLY); 
	gpio_set_intr_type(BTN_CONTROL_ON_OFF, GPIO_INTR_DISABLE);

	char buffer_lcd_0_task[16];
	char buffer_lcd_1_task[16];
	while(1)
    {
		if(!gpio_get_level(RE_BTN_GPIO))
		{
			ok_set_parameters = true;
			vTaskDelay(200/portTICK_PERIOD_MS); 
			hd44780_clear(&lcd_monitor); 
			vTaskDelay(200/portTICK_PERIOD_MS);

			while(ok_set_parameters)
			{	
				if(!gpio_get_level(BTN_CONTROL_ON_OFF))
				{
					set_mode_ToM = !set_mode_ToM;
					vTaskDelay(200/portTICK_PERIOD_MS);
					hd44780_clear(&lcd_monitor); 
					vTaskDelay(200/portTICK_PERIOD_MS);
				}

				for(int st=0; st<16; st++){buffer_lcd_0_task[st]='\0'; buffer_lcd_1_task[st]='\0';}

				if(set_mode_ToM)//true->Set Temperature
				{
					hd44780_gotoxy(&lcd_monitor, 0, 0);
					sprintf(buffer_lcd_0_task, "Temp:%.0f \337C  ", refTemp);
        			hd44780_puts(&lcd_monitor, buffer_lcd_0_task);

					hd44780_gotoxy(&lcd_monitor, 0, 1);
					sprintf(buffer_lcd_1_task, " ");
        			hd44780_puts(&lcd_monitor, buffer_lcd_1_task);

					code <<= 2;
    				code |= gpio_get_level(RE_A_GPIO);
   					code |= gpio_get_level(RE_B_GPIO) << 1;
    				code &= 0xf;

					if(valid_states[code])
					{
						inc = 0;

    					store = (store << 4) | code;
    					if(store == 0xe817) inc = 1;
    					if(store == 0xd42b) inc = -1;

						if((refTemp+inc)>=0 && (refTemp+inc)<=230)
    					{
							refTemp += inc;
							for(int st=0; st<16; st++){buffer_lcd_0_task[st]='\0'; buffer_lcd_1_task[st]='\0';}
							hd44780_gotoxy(&lcd_monitor, 0, 0);
							sprintf(buffer_lcd_0_task, "Temp:%.0f \337C  ", refTemp);
        					hd44780_puts(&lcd_monitor, buffer_lcd_0_task);

							hd44780_gotoxy(&lcd_monitor, 0, 1);
							sprintf(buffer_lcd_1_task, " ");
        					hd44780_puts(&lcd_monitor, buffer_lcd_1_task);
    					}
					}
				}
				else//false->Set Motor Velocity	***EN PRUEBAS EL PWM***
				{
					hd44780_gotoxy(&lcd_monitor, 0, 0);
					sprintf(buffer_lcd_0_task, "MOTOR DUTY:%.0f ", dFactorLS*100.0f);
        			hd44780_puts(&lcd_monitor, buffer_lcd_0_task);

					hd44780_gotoxy(&lcd_monitor, 0, 1);
					sprintf(buffer_lcd_1_task, " ");
        			hd44780_puts(&lcd_monitor, buffer_lcd_1_task);

					code <<= 2;
    				code |= gpio_get_level(RE_A_GPIO);
   					code |= gpio_get_level(RE_B_GPIO) << 1;
    				code &= 0xf;

					if(valid_states[code])
					{
						inc_f = 0.0f;

    					store = (store << 4) | code;
    					if(store == 0xe817) inc_f = 0.10f;
    					if(store == 0xd42b) inc_f = -0.10f;

						if((dFactorLS+inc_f)>=0.0f && (dFactorLS+inc_f)<=1.0f)
    					{
							dFactorLS += inc_f;
							for(int st=0; st<16; st++){buffer_lcd_0_task[st]='\0'; buffer_lcd_1_task[st]='\0';}
							hd44780_gotoxy(&lcd_monitor, 0, 0);
							sprintf(buffer_lcd_0_task, "MOTOR DUTY:%.0f ", dFactorLS*100.0f);
        					hd44780_puts(&lcd_monitor, buffer_lcd_0_task);

							hd44780_gotoxy(&lcd_monitor, 0, 1);
							sprintf(buffer_lcd_1_task, " ");
        					hd44780_puts(&lcd_monitor, buffer_lcd_1_task);
							set_PWM_Duty_LS();
    					}
					}
				}
				if(!gpio_get_level(RE_BTN_GPIO))
				{
					vTaskDelay(200/portTICK_PERIOD_MS); 
					hd44780_clear(&lcd_monitor); 
					vTaskDelay(200/portTICK_PERIOD_MS); 
					ok_set_parameters = false;
				}
			}
			set_mode_ToM = true;
		}

		if(!gpio_get_level(BTN_CONTROL_ON_OFF))
		{
			vTaskDelay(200/portTICK_PERIOD_MS);
			control_ON = !control_ON;
			vTaskDelay(200/portTICK_PERIOD_MS);
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) 
    {
        printf("eFuse Two Point: Supported\n");
    } 
    else 
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    } 
    else 
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if(val_type == ESP_ADC_CAL_VAL_EFUSE_TP) 
    {
        printf("Characterized using Two Point Value\n");
    } 
    else
    {
        if(val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) 
        {
            printf("Characterized using eFuse Vref\n");
        } 
        else 
        {
            printf("Characterized using Default Vref\n");
        }
    }
}

void setup_ADC(void)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

void adc_read(void)
{
	adc_raw = 0;
    for(int i = 0; i < NO_OF_SAMPLES; i++) 
    {
        adc_raw += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_raw /= NO_OF_SAMPLES;
    voltage = esp_adc_cal_raw_to_voltage(adc_raw, adc_chars);
	voltage = kV*voltage + (1 - kV)*voltage_x1;//Filter
	voltage_x1 = voltage;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup_PWM(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_BIT, //Resolution of PWM duty
        .freq_hz = freqHS,                 //Frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,        //Timer mode
        .timer_num = LEDC_HS_TIMER         //Timer index
    };
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));//Set configuration of timer0 for high speed channels
        
	//Prepare and set configuration of timer1 for low speed channels
    ledc_timer.freq_hz = freqLS;
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for(int ch = 0; ch < 3; ch++) 
    {
        ledc_channel_config(&ledc_channel[ch]);
    }
}

void set_PWM_Duty_HS(void)
{
	dutyHS = bitDec * dFactorHS;
	dutyHS = round(dutyHS);
	for(int ch = 0; ch < 2; ch++) 
	{
    	ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, dutyHS);
    	ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	}
}

void set_PWM_Duty_LS(void)
{
	dutyLS = bitDec * dFactorLS;
	dutyLS = round(dutyLS);
    ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, dutyLS);
    ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);
}

void set_PWM_Frequency(void)//
{
	freqHS = round(freqHS);      
	ledc_set_freq(LEDC_HS_MODE, LEDC_HS_TIMER, freqHS);

	freqLS = round(freqLS);      
	ledc_set_freq(LEDC_LS_MODE, LEDC_LS_TIMER, freqLS);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

void setup_LCD()
{
	ESP_ERROR_CHECK(i2cdev_init());

	hd44780_t lcd = {
        .write_cb = write_lcd_data,//Use callback to send data to LCD by I2C GPIO expander
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = 0,
            .e  = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3
        }
    };
	lcd_monitor = lcd;

	memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, 0, I2C_ADDR, SDA_GPIO, SCL_GPIO));

    ESP_ERROR_CHECK(hd44780_init(&lcd_monitor));

    hd44780_switch_backlight(&lcd_monitor, true);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void getTemp(void)
{
	adc_read();
	rt = (vcc/(float)voltage)*res - res; 
	loga = log(rt);
	invT = aSH + (bSH * loga) + (cSH * loga * loga * loga);
	temp = (1/invT) - 273.15;
	temp = kT*temp + (1 - kT)*temp_x1;//Filter
	temp_x1 = temp;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main_GUI(void)
{
	char uart_recv[] = {'\0', '\0', '\0', '\0'};
	if(wifi_OK)
	{
		recv_flag = recvfrom(sock, buffer_recv, sizeof(buffer_recv) - 1, MSG_DONTWAIT, (struct sockaddr *)&source_addr, &socklen);
    	if(recv_flag > 0)
    	{
        	buffer_recv[recv_flag] = 0;
			charIN = buffer_recv[0];
    	}		
	}
	else
	{
		gets(uart_recv);
		if(uart_recv[0] != '\0')
		{
			charIN = uart_recv[0];
		}
		fflush(stdin);
	}

	if(charIN == 'y')
	{	
		charIN = 'x';	

		if(wifi_OK)
		{
			buffer_recv[0] = '0';
			numIN = atoi(buffer_recv);
		}
		else
		{
			uart_recv[0] = '0';
			numIN = atoi(uart_recv);
		}
		refTemp = numIN;
		led_GUI = true;
	}
	else
	{
		if(charIN == 'z')
		{
			charIN = 'x';
			control_ON = !control_ON;
			led_GUI = true;
		}
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
float limiter(float amt, float low, float high)
{
	if(amt < low)
		return(low);
	else
		if(amt > high)
			return(high);
		else
			return(amt);
}

void reset_values(void)
{
	integratedError = 0.0f; 
 	lastError = 0.0f;
}

void control_PID(float target, float current, float deltaTime)
{
	error = (target - current);
  
  	pPart = kP * error;
  
  	integratedError += error * deltaTime;    
  	integratedError = limiter(integratedError, -iLIMIT, iLIMIT);
  	iPart = kI * integratedError; 
  
  	dPart = (kD / deltaTime) * (error - lastError);    
  	lastError = error;
   
  	pidValue = pPart + iPart + dPart; 
}

void control(void)
{	
	main_GUI();
	if(flow_time_PID >= 0.4f)//400 ms
	{
		if(control_ON == true)
		{
			control_PID(refTemp, temp, flow_time_PID);
			dFactorHS = limiter(pidValue, 0.0f, 1.0f);
			set_PWM_Duty_HS();
			flow_time_PID = 0.0f;
		}
		else
		{
			reset_values();
			dFactorHS = 0.0f;
			set_PWM_Duty_HS();
			flow_time_PID = 0.0f;
		}
	}
	/*
	La onda sinusoidal hace 24 ciclos 
	durante cada periodo de la ley de control
	*/
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main_msg(void)
{
	if(flow_time_UDP > 0.05f && wifi_OK)//20 Hz -> 50 ms
	{
		sprintf(buffer_send, "%.1f,%.1f,%.4f\n", refTemp, temp, dFactorHS);
		msg_UDP();
		flow_time_UDP = 0.0f;	
	}

	if(flow_time_UART > 0.05f && !wifi_OK) 
	{
		printf("%.1f,%.1f,%.4f\n", refTemp, temp, dFactorHS);
		flow_time_UART = 0.0f;
	}

	if(flow_time_LCD > 0.5f && !ok_set_parameters) 
	{
		char buffer_lcd_0[16];
		char buffer_lcd_1[16];

		hd44780_gotoxy(&lcd_monitor, 0, 0);
		sprintf(buffer_lcd_0, "Temp:%.0f/%.0f \337C  ", temp, refTemp);
        hd44780_puts(&lcd_monitor, buffer_lcd_0);

		hd44780_gotoxy(&lcd_monitor, 0, 1);
		if(control_ON){sprintf(buffer_lcd_1, "CONTROL ON ");} else{sprintf(buffer_lcd_1, "CONTROL OFF");}
        hd44780_puts(&lcd_monitor, buffer_lcd_1);
		flow_time_LCD = 0.0f;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void app_main(void)
{
    setup_ADC();
	setup_LCD();
	setup_PWM();
	setup_BLINK();
	setup_WIFI_UDP();
	
	blink();//Setup OK

	xTaskCreatePinnedToCore(blink_task, "blink_task", 2000, NULL, 1, NULL, 1);//CPU 1 SE USA CON FREERTOS PARA MULTI TAREA
	xTaskCreatePinnedToCore(encoder_task, "encoder_task", configMINIMAL_STACK_SIZE * 8, NULL, 2, NULL, 1);

    gettimeofday(&tvalBefore, NULL);
    while(1)//CPU 0 SE USA CON PROGRAMACION SECUENCIAL 
    {
		getTemp();
		control();
        main_msg();
		get_dt(false);
        feedTheDog();
    }
}