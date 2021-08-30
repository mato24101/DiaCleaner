#include "pwmOut.h"

pwmOut::pwmOut(gpio_num_t gpio, ledc_channel_t channel, ledc_timer_t timer, uint16_t freq) {
    ledc_timer_config_t ledc_timer;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = freq;
    ledc_timer.speed_mode = PWM_SPEED_MODE;
    ledc_timer.timer_num = timer;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret) printf("Timer init failed with ret: %d\r\n", ret);

    this -> pwm_config.channel    = channel;
    this -> pwm_config.duty       = 0;
    this -> pwm_config.gpio_num   = gpio;
    this -> pwm_config.speed_mode = PWM_SPEED_MODE;
    this -> pwm_config.hpoint     = 0;
    this -> pwm_config.timer_sel  = timer;
    this -> pwm_config.intr_type  = LEDC_INTR_DISABLE;

    ret = ledc_channel_config(&this -> pwm_config);
    if (ret) printf("PWM config failed with ret: %d\r\n", ret);
}//pwmOut

esp_err_t pwmOut::setDuty(float duty) {
    uint16_t pwmDuty = (uint16_t)round(PWM_TIMER_RESOLUTION * duty);

    esp_err_t ret = ledc_set_duty(this -> pwm_config.speed_mode, this -> pwm_config.channel, pwmDuty);
    if (ret) return ret;

    return ledc_update_duty(this -> pwm_config.speed_mode, this -> pwm_config.channel);
}//setDuty