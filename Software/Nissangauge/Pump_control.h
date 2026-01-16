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
#define Out_aux_pwm 5 //Output AUX pump PWM

//Temperatures//

#define heatup_temp 25  //temperatur  difference to target temperature (cold engine)
#define heatup_temp2 10  //temperatur  difference to target temperature (not so cold engine)
#define target_temp 85 //target cooling temperature
#define start_fan_diff_temp 7 //target_temp + start_fan_diff_temp = Fan strat temp
#define stop_fan_diff_temp 3 //target_temp + stop_fan_diff_temp = Fan stop temp
#define over_temp 13 //target_temp + start_fan_diff_temp = Start limpmode (Pump 100% Fan=On)

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
#define aux_low 150 //PWM value for aux pump low speed
#define aux_norm 216  //PWM value for aux pump norm speed
#define aux_hot 255 //PWM value for aux pump hot speed
#define aux_min 90 //PWM value for aux pump absolute minimum speed


//times
#define lowspeed_time 30000 //pulse time warmupmode lowspeed (ms)
#define highspeed_time 4000 //pulse time warmupmode highspeed (ms)
#define afterrun_time 600 //time (1/10s) for afterrun when ignition off
#define blink_time 10 //time (1/10s) for additonal blink events

//PID constants
#define kp -50
#define ki -40
#define kd -130
#define i_loops 10 //1000ms
#define d_loops 50 //3000ms

//LED brightness
#define led_g_min 5   //5
#define led_g_max 50  //50
#define led_g_s2 15   //15
#define led_b_min 15   //15
#define led_b_max 25   //25


void initPump(); 
uint16_t get_pot(uint8_t a_input, uint8_t pwm1, uint8_t pwm2);
uint16_t get_pot_duty(uint8_t a_input);
uint16_t test_pot_duty(uint8_t a_input);
uint8_t get_pump_duty();
uint8_t get_aux_duty();
bool get_fan_state();
bool get_afterrun();
bool get_ign_state();

bool get_overtemp();
bool get_Temp_sens_state(uint8_t cltTemp);
void runPump();
void Fan();
bool afterrun_ok();
void afterrun_condition (bool (*condition)(void));
void afterrun_control();
void set_pumpspeed (uint8_t pumpspeed);
void set_auxspeed (uint8_t auxspeed);
uint16_t GETDATA (uint8_t x);
int16_t get_PWM (uint16_t clt, uint16_t low_point, uint16_t high_point, bool run);
void testmode_init();
void testmode();
