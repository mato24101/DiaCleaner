
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "mcp23008.h"
#include "driver/i2c.h"
#include "mcp23009.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define LED_GPIO (gpio_num_t)2
#define OnOff_BDD1 15
#define OnOff_BDD2 13

#define MCP_SET_BIT(x) mcp23008_write_port(&conf, conf.current |= (1 << x));
#define MCP_CLR_BIT(x) mcp23008_write_port(&conf, conf.current &= ~(1 << x)); //conf.current & (0 << x)

mcp23009 mcp; //initialize mcp object

TaskHandle_t task = NULL;

static bool ISR(void *)
{
    BaseType_t taskWoken;
    xTaskGenericNotifyFromISR(task, 1, eSetValueWithOverwrite, 0, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
    return true;
}
/*
void taskk(void *)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, 100);
        //mcp.setBit(M_GPIO,0);
        printf("Turning ON the LED\n");
        //gpio_set_level(LED_GPIO, 1);
        ulTaskNotifyTake(pdTRUE, 100);
        //mcp.clrBit(M_GPIO, 0);
        printf("Turning OFF the LED\n");
       // gpio_set_level(LED_GPIO, 0);
    }
}
*/
extern "C" void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    //config
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

    mcp.init(&mcp_conf); //initializes variables and reads all registers
/*
    timer_config_t tim_conf = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,
    };

    xTaskCreate(taskk, "Duty", 4096, NULL, 2, &task);
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &tim_conf));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 100000));
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ISR, NULL, 0);
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
    */
    //set whole GPIO port in one call
    mcp.setDir(0xFF);

    //read GPIO port state to variable val
    uint8_t val;
    mcp.getDir(&val);
    ESP_LOGI("GPIOs", "0x%x", val);

    //enable pullups
    mcp.setPullup(0xFF);
    //verify
    mcp.getPullup(&val);
    ESP_LOGI("PULLups", "0x%x", val);

    //set just 3rd bit
    //mcp.setBit(M_OLAT,3);
    //mcp.clrBit(M_OLAT,3);

    //arduino style gpio control, switch 2nd GPIO bit to 1
    //mcp.digitalWrite(2,1);
    mcp.setBit(M_GPIO, 4);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    mcp.clrBit(M_GPIO, 4);
    while (1)
    {
        /* Blink off (output low) 
        printf("Turning ON the LED\n");
        gpio_set_level(LED_GPIO, 1);
*/
        mcp.setBit(M_GPIO, 0);
        mcp.setBit(M_OLAT, 1);
        mcp.setBit(M_OLAT, 2);
        mcp.setBit(M_OLAT, 3);
        mcp.setBit(M_OLAT, 4);
        //mcp.setBit(M_GPIO, 5);
        //mcp.setBit(M_GPIO, 6);
        //mcp.setBit(M_GPIO, 7);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        /*
        printf("Turning OFF the LED\n");
        gpio_set_level(LED_GPIO, 0);
        */

        mcp.clrBit(M_GPIO, 0);
        mcp.clrBit(M_OLAT, 1);
        mcp.clrBit(M_OLAT, 2);
        mcp.clrBit(M_OLAT, 3);
        mcp.clrBit(M_OLAT, 4);
        //mcp.clrBit(M_GPIO, 5);
        //mcp.clrBit(M_GPIO, 6);
        //mcp.clrBit(M_GPIO, 7);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
