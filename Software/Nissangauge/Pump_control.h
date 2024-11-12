//#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include "Arduino.h"

//INPUT//

#define In_low_ref A1 //ref Potentiometer specific Minimum pump speed
#define In_high_ref A0 //ref Potentiometer specific Maximum pump speed


#define In_run 13 //Input Pin Ignition for afterrun function

//Output//

#define Out_led_r 9 //Output Pin contorl LED Red
#define Out_led_g 11 //Output Pin contorl LED Green
#define Out_led_b 10 //Output Pin contorl LED Blue
#define Out_afterrun_on 8 //Output Pin power
#define Out_pump_pwm 3  //Output Pin CLT pump
#define Out_fan 7 //Output Pin cooling fan

//Temperatures//

#define heatup_temp 23  //temperatur  difference to target temperature (cold engine)
#define heatup_temp2 18  //temperatur  difference to target temperature (not so cold engine)
#define target_temp 88 //target cooling temperature
#define start_fan_diff_temp 6 //target_temp + start_fan_diff_temp = Fan strat temp
#define stop_fan_diff_temp 2 //target_temp + stop_fan_diff_temp = Fan stop temp
#define over_temp 11 //target_temp + start_fan_diff_temp = Start limpmode (Pump 100% Fan=On)

//Speeds
#define no_speed 36 //PWM value for 0% pumpspeed
#define hot_speed 215 //207 PWM 95% pumpspeed (for Limpmode or cooldown mode)
#define max_speed 216 //PWM 36-216 = 0-100% pumpspeed (limit maximum pumpspeed)
#define lowspeed 54 //PWM pump lowspeed 8% = 350rpm (limit minimum pumpspeed)
#define midspeed 126 //PWM pump speed 50% = 2250rpm (pumpspeed cooldown)
#define tps_limit 90 //TPS setpoint to add more flow
#define rpm_limit 6000 //RPM setpoint to add more flow
#define add_rpm_v 0 //PWM add RPM (Value added to pump speed when RPM limit is exceeded)
#define add_tps_v 0 //PWM add TPS (Value added to pump speed when TPS limit is exceeded)


//times
#define lowspeed_time 30000 //pulse time warmupmode lowspeed (ms)
#define highspeed_time 4000 //pulse time warmupmode highspeed (ms)
#define afterrun_time 600 //time (1/10s) for afterrun when ignition off
#define blink_time 10 //time (1/10s) for additonal blink events

//PID constants
#define kp -50
#define ki -40
#define kd -50
#define i_loops 10 //1000ms
#define d_loops 20 //3000ms

//LED brightness
#define led_g_min 5   //5
#define led_g_max 250  //50
#define led_g_s2 150   //15
#define led_b_min 150   //15
#define led_b_max 250   //25


void initPump(); 
uint16_t get_pot(uint8_t a_input, uint8_t pwm1, uint8_t pwm2);
uint16_t get_pot_duty(uint8_t a_input);
uint8_t get_pump_duty();
bool get_fan_state();
bool get_afterrun();
bool get_ign_state();
bool get_Temp_sens_state(uint8_t cltTemp);
void runPump();
void Fan();
bool afterrun_ok();
void afterrun_condition (bool (*condition)(void));
void afterrun_control();
void set_pumpspeed (uint8_t pumpspeed);
uint16_t GETDATA (uint8_t x);
int16_t get_PWM (uint16_t clt, uint16_t low_point, uint16_t high_point, bool run);
