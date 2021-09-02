#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "mcp23008.h"
#include "driver/i2c.h"
#include "mcp23009.h"
#include "driver/timer.h"
#include "pwmOut.h"
/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
//LED
#define LED_GPIO GPIO_NUM_2
#define Blink_time 1 //s
#define LED_PWM_FREQ 5000 //Hz
#define BDD1_Duty GPIO_NUM_26
#define BDD2_Duty 

//System
#define S_TO_MS 1000
#define S_TO_US 1000000
#define POSITIVE true
#define NEGATIVE false

//I2C config
#define SDA_GPIO GPIO_NUM_21
#define SCL_GPIO GPIO_NUM_22
#define PORT 0

//BDD1 config
#define BDD1_MTIME 2      //s   //measure
#define BDD1_POLTIME 2    //s   //polarity
#define BDD1_DC 0.5       //x100%
#define BDD1_PWM_FREQ 500 //Hz
#define BDD1_POL_GPIO 4
#define BDD1_DUTY_GPIO GPIO_NUM_26
#define BDD1_DEFAULT_POL NEGATIVE

//BDD2 config
#define BDD2_MTIME 2     //s   //measure
#define BDD2_POLTIME 1   //s   //polarity
#define BDD2_DC 0.5      //x100%
#define BDD2_PWM_FREQ 2000 //Hz
#define BDD2_POL_GPIO 5
#define BDD2_DUTY_GPIO GPIO_NUM_27
#define BDD2_DEFAULT_POL NEGATIVE

//Rele config
#define Rele1   3
#define Rele2   2
#define Rele3   1
#define Rele4   0

//static int64_t lastMeasure = 0;
static int64_t lastPol_BDD1 = 0;
static int64_t lastPol_BDD2 = 0;
static int64_t lastDuty_BDD1 = 0;
static int64_t lastDuty_BDD2 = 0;
static int64_t lastBlink_state = 0;
static bool polarity_BDD1 = BDD1_DEFAULT_POL;
static bool polarity_BDD2 = BDD2_DEFAULT_POL;
static bool Duty_polBDD1 = 0;
static bool Duty_polBDD2 = 0;
static bool Blink_state = 0;

mcp23009 mcp; //initialize mcp object

pwmOut pwm_BDD1(BDD1_DUTY_GPIO, LEDC_CHANNEL_0, LEDC_TIMER_0, BDD1_PWM_FREQ);
pwmOut pwm_BDD2(BDD2_DUTY_GPIO, LEDC_CHANNEL_1, LEDC_TIMER_1, BDD2_PWM_FREQ);


void Blink()
{

    if ((esp_timer_get_time() - lastBlink_state) >= (Blink_time * S_TO_US))
    {
        Blink_state = !Blink_state;
        gpio_set_level(LED_GPIO, Blink_state);
        printf("im OK %d\n", Blink_state);
        //gpio_set_level(BDD1_DUTY_GPIO, LED_state);
        //gpio_set_level(BDD2_DUTY_GPIO, LED_state);
        mcp.digitalWrite(Rele4, Blink_state);
        lastBlink_state = esp_timer_get_time();
    }
}

void polTime_BDD1()
{
    if ((esp_timer_get_time() - lastPol_BDD1) >= (BDD1_POLTIME * S_TO_US))
    {
        polarity_BDD1 = !polarity_BDD1;
        mcp.digitalWrite(BDD1_POL_GPIO, polarity_BDD1);
        printf("BBD1 set to %d\n", polarity_BDD1);
        lastPol_BDD1 = esp_timer_get_time();
    } //if ((esp_timer_get_time() - lastPol) >= (BDD_POLTIME * S_TO_US))
} //polTime

void polTime_BDD2()
{
    if ((esp_timer_get_time() - lastPol_BDD2) >= (BDD2_POLTIME * S_TO_US))
    {
        polarity_BDD2 = !polarity_BDD2;
        mcp.digitalWrite(BDD2_POL_GPIO, polarity_BDD2);
        printf("BBD2 set to %d\n", polarity_BDD2);
        lastPol_BDD2 = esp_timer_get_time();
    } //if ((esp_timer_get_time() - lastPol) >= (BDD_POLTIME * S_TO_US))
} //polTime

void Duty_FOR()
{
    for (float i = 0.1; i <= 100; i=i+10)
        {
            float duty = i / 100;

            pwm_BDD1.setDuty(duty);
            pwm_BDD2.setDuty(duty);
            //pwm_LED.setDuty(duty);
            printf("pwmDuty %.2f\n", duty);
            vTaskDelay(500 / portTICK_RATE_MS);
            //pwm.pwmOut(LED_GPIO,0,10,BDD1_PWM_FREQ);
        }
}
void PWM_Init()
{
    pwmOut pwm_BDD1(BDD1_DUTY_GPIO, LEDC_CHANNEL_0, LEDC_TIMER_0, BDD1_PWM_FREQ);
    pwmOut pwm_BDD2(BDD2_DUTY_GPIO, LEDC_CHANNEL_1, LEDC_TIMER_1, BDD2_PWM_FREQ);
}
    

extern "C" void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDD1_DUTY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDD2_DUTY_GPIO, GPIO_MODE_OUTPUT);
    //pwmOut pwm_LED(LED_GPIO, LEDC_CHANNEL_0, LEDC_TIMER_0, LED_PWM_FREQ);
    //pwm_LED.setDuty(1);
    PWM_Init();
    //pwmOut pwm_BDD1(BDD1_DUTY_GPIO, LEDC_CHANNEL_0, LEDC_TIMER_0, BDD1_PWM_FREQ);
    //pwmOut pwm_BDD2(BDD2_DUTY_GPIO, LEDC_CHANNEL_1, LEDC_TIMER_1, BDD2_PWM_FREQ);
   
    mcpxx9_conf_t mcp_conf = {
        .i2c_port = I2C_NUM_0,
        .i2c_conf = {
            //i2c driver config
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_NUM_21,
            .scl_io_num = GPIO_NUM_22,
            //.master.clk_speed=10000,
        },
        .mcp_addr = 0x20, //address of mcp23009
    };

    mcp_conf.i2c_conf.master.clk_speed = 10000; //clk speed

    //polInit
    mcp.init(&mcp_conf);                               //initializes variables and reads all registers
    mcp.setDir(0x00);                                  //seting direction to output
    mcp.setPullup(0xFF);                               //enable pullups
    mcp.digitalWrite(BDD1_POL_GPIO, BDD1_DEFAULT_POL); //default polarity BDD1
    mcp.digitalWrite(BDD2_POL_GPIO, BDD2_DEFAULT_POL); //default polarity BDD2

    /*
    while (1)
    {
        printf("BDD START\r\n");
        mcp.digitalWrite(0,1);
        printf("BBD set");
        vTaskDelay(5000 / portTICK_RATE_MS);

        mcp.digitalWrite(0,0);
        printf("BBD reset");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
   
    lastPol_BDD1 = esp_timer_get_time();
    lastPol_BDD2 = esp_timer_get_time();
    */

    while (true)
    {
        //printf("im OK");
        //inaMeasure();
        //polTime_BDD1();
        //polTime_BDD2();
        // Duty_BDD1();
        // Duty_BDD2();
        Blink();
        //Duty_FOR();
        //pwm.setDuty(0.5);
        /*

        for (float i = 0; i <= 100; i=i+10)
        {
            float duty = i / 100;

            pwm_BDD1.setDuty(duty);
            pwm_BDD2.setDuty(duty);
            //pwm_LED.setDuty(duty);
            printf("pwmDuty %.2f\n", duty);
            vTaskDelay(500 / portTICK_RATE_MS);
            //pwm.pwmOut(LED_GPIO,0,10,BDD1_PWM_FREQ);
        }
        */

       /*
        printf("im ON\n");
        gpio_set_level(LED_GPIO, 1);
        //gpio_set_level(BDD1_Duty, 1);
        //gpio_set_level(BDD2_Duty, 1);
        vTaskDelay(1000 / portTICK_RATE_MS);
        printf("im OFF\n");
        gpio_set_level(LED_GPIO, 0);
        //gpio_set_level(BDD1_Duty, 0);
        //gpio_set_level(BDD2_Duty, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        */
       vTaskDelay(100 / portTICK_RATE_MS);
    } //while (true)
}

