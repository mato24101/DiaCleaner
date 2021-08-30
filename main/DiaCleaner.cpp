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
#define LED_GPIO (gpio_num_t)2
#define LED_blink_time 1 //s
#define OnOff_BDD1 15
#define OnOff_BDD2 13

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
#define BDD1_MTIME 2     //s   //measure
#define BDD1_POLTIME 2   //s   //polarity
#define BDD1_DC 0.5      //*100%
#define BDD1_PWM_FREQ 200 //Hz
#define BDD1_POL_GPIO 4
#define BDD1_DUTY_GPIO 6
#define BDD1_DEFAULT_POL NEGATIVE

//BDD2 config
#define BDD2_MTIME 2     //s   //measure
#define BDD2_POLTIME 1   //s   //polarity
#define BDD2_DC 0.5      //*100%
#define BDD2_PWM_FREQ 10 //Hz
#define BDD2_POL_GPIO 5
#define BDD2_DUTY_GPIO 7
#define BDD2_DEFAULT_POL NEGATIVE

//static int64_t lastMeasure = 0;
static int64_t lastPol_BDD1 = 0;
static int64_t lastPol_BDD2 = 0;
static int64_t lastDuty_BDD1 = 0;
static int64_t lastDuty_BDD2 = 0;
static int64_t lastLED_state = 0;
static bool polarity_BDD1 = BDD1_DEFAULT_POL;
static bool polarity_BDD2 = BDD2_DEFAULT_POL;
static bool Duty_polBDD1 = 0;
static bool Duty_polBDD2 = 0;
static bool LED_state = 0;

mcp23009 mcp; //initialize mcp object

void Blink()
{
    if ((esp_timer_get_time() - lastLED_state) >= (LED_blink_time * S_TO_US))
    {
        LED_state = !LED_state;
        gpio_set_level(LED_GPIO, LED_state);
        lastLED_state = esp_timer_get_time();
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
/*
void Duty_BDD1()
{
    if ((esp_timer_get_time() - lastDuty_BDD1) >= ((1 / BDD1_PWM_FREQ) * S_TO_US))
    {
        Duty_polBDD1 = !Duty_polBDD1;
        mcp.digitalWrite(BDD1_DUTY_GPIO, Duty_polBDD1);
        //printf("BBD1 set to %d\n", polarity_BDD1);
        lastDuty_BDD1 = esp_timer_get_time();
    } //if ((esp_timer_get_time() - lastPol) >= (BDD_POLTIME * S_TO_US))
} //polTime

void Duty_BDD2()
{
    if ((esp_timer_get_time() - lastDuty_BDD2) >= ((1 / BDD2_PWM_FREQ) * S_TO_US))
    {
        Duty_polBDD2 = !Duty_polBDD2;
        mcp.digitalWrite(BDD2_DUTY_GPIO, Duty_polBDD2);
        //printf("BBD1 set to %d\n", );
        lastDuty_BDD2 = esp_timer_get_time();
    } //if ((esp_timer_get_time() - lastPol) >= (BDD_POLTIME * S_TO_US))
} //polTime
*/
extern "C" void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    pwmOut pwm(LED_GPIO, LEDC_CHANNEL_0, LEDC_TIMER_0, BDD1_PWM_FREQ);

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

    mcp_conf.i2c_conf.master.clk_speed = 100000; //clk speed

    //polInit
    mcp.init(&mcp_conf); //initializes variables and reads all registers
    mcp.setDir(0x00);    //seting direction to output
    mcp.setPullup(0xFF); //enable pullups

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
    */

    lastPol_BDD1 = esp_timer_get_time();
    lastPol_BDD2 = esp_timer_get_time();

    while (true)
    {
        //inaMeasure();
        polTime_BDD1();
        polTime_BDD2();
        // Duty_BDD1();
        // Duty_BDD2();
        //Blink();
        //pwm.setDuty(0.5);
        
        for (float i = 0; i <= 100 ; i++)
        {
            float duty = i/100;

            pwm.setDuty(duty);
            printf("pwmDuty %.2f\n", duty);
            vTaskDelay(100 / portTICK_RATE_MS);
            //pwm.pwmOut(LED_GPIO,0,10,BDD1_PWM_FREQ);
        }
        
        vTaskDelay(100 / portTICK_RATE_MS);

    } //while (true)
}
