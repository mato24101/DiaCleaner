#ifndef PWM_OUT_H_
#define PWM_OUT_H_

#include <stdio.h>
#include <math.h>
#include <esp_system.h>

#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define PWM_SPEED_MODE          LEDC_LOW_SPEED_MODE
#define PWM_TIMER_RESOLUTION    8192

#define PWM_DEFAULT_CHANNEL LEDC_CHANNEL_0
#define PWM_DEFAULT_TIMER   LEDC_TIMER_0
#define PWM_DEFAULT_FREQ    5000//Hz

/**
 * \defgroup drivers_PwmOut PwmOut class
 * \ingroup drivers-public-api-gpio
 * @{
 */

/**
 * @brief A PWM digital output
 */
class pwmOut{
public:

    /**
     * @brief Creates a PWM output connected to the given gpio pin
     * 
     * @param gpio GPIO pin number
     * @param channel LEDC peripheral channel number
     * @param timer timer number
     * @param freq PWM signal frequency
     */
    pwmOut(gpio_num_t gpio, ledc_channel_t channel, ledc_timer_t timer, uint16_t freq);

    /**
     * @brief Sets the PWM output duty cycle
     * 
     * @param duty duty cycle
     * 
     * @return
     *      ESP_OK on success
     *      ESP_ERR otherwise
     */
    esp_err_t setDuty(float duty);

    /**
     * @brief An operator shorthand for setDuty
     */
    pwmOut &operator= (float value) {
        setDuty(value);
        return *this;
    }
    
private:
    ledc_channel_config_t pwm_config;//pwm channel LEDC peripheral configuration
};

/** @}*/

#endif//PWM_OUT_H_