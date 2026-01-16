#include "Arduino.h"
#include "Pump_control.h"
#include "Pages.h"


bool after_run = false;
bool warm_dis = false;
uint8_t get_CLT = 0;
uint8_t pump_duty = 0;
uint8_t aux_duty = 0;
bool fan_on = false;
int16_t PID;
//uint16_t auxspeed_2 = 0;

void initPump() 
{
  //declare Pins
  pinMode(Out_pump_pwm, OUTPUT);
  pinMode(Out_led_r, OUTPUT);
  pinMode(Out_afterrun_on, OUTPUT);
  pinMode(Out_fan, OUTPUT);

  pinMode(In_run, INPUT);
  
  //pump start up see CWA200 pump docs
  // simulate LIN-Bus commands to wakeup and setup PWM mode 
  analogWrite(Out_pump_pwm, 0); //100% PWM Pumpspeed
  digitalWrite(Out_led_r, HIGH); //LED indicator
  delay(300); //for 300ms
  analogWrite(Out_pump_pwm, 255);
  digitalWrite(Out_led_r, LOW); //LED indicator
  delay(100); //wait 100ms before reset pump
  analogWrite(Out_pump_pwm, 229); // 10% duty cycle to reset it
  digitalWrite(Out_led_r, HIGH); //LED indicator
  delay(300); // for 300ms
  analogWrite(Out_pump_pwm, 255);
  digitalWrite(Out_led_r, LOW); //LED indicator
  delay(10);
  
}

uint16_t get_pot(uint8_t a_input, uint8_t pwm1, uint8_t pwm2) // get values from the desired potentiometer
{
  uint16_t sensorValue;
  sensorValue = map(analogRead(a_input), 0, 1023, pwm1, pwm2);
  return sensorValue;
}

uint16_t get_pot_duty(uint8_t a_input) //for display return the current position of potentiometer
{
  uint16_t high_pot_duty;
  uint16_t low_pot_duty;
  low_pot_duty = map(analogRead(In_low_ref), 0, 1023, 100 * (lowspeed - no_speed)/(max_speed - no_speed), 100 * (hot_speed - no_speed)/(max_speed - no_speed));
  high_pot_duty = map(analogRead(In_high_ref), 0, 1023, low_pot_duty, 100);

  if (a_input == false)
  {
    return low_pot_duty;
  }
  else
  {
    return high_pot_duty;
  }
}

uint16_t test_pot_duty(uint8_t a_input) //for display return the current position of potentiometer in testmode
{
  uint16_t high_pot_duty_t;
  uint16_t low_pot_duty_t;
  low_pot_duty_t = map(analogRead(In_low_ref), 0, 1023, 0, 100);
  high_pot_duty_t = map(analogRead(In_high_ref), 0, 1023, 0, 100);

  if (a_input == false)
  {
    return low_pot_duty_t;
  }
  else
  {
    return high_pot_duty_t;
  }
}


uint8_t get_pump_duty() //for display return the current  duty of the pump
{
  return pump_duty;    
}

uint8_t get_aux_duty() //for display return the current  duty of the auxpump
{
  return aux_duty;    
}

bool get_fan_state() //for display get the actually state of the cooling fan
{
  if (digitalRead(Out_fan) == true)
  {
    return 1;
  } 
  else
  { 
    return 0;
  } 
}

bool get_afterrun() // for display get the state of the after run output 
{
  return after_run;
}

bool get_ign_state() // for display get the state of the engine is running 
{
  if (digitalRead(In_run) == true)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

bool get_warmup() // for display return the state of the warmup mode
{
  return warm_dis;
}

bool get_overtemp()
{
  if (get_CLT > target_temp + over_temp)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool get_Temp_sens_state(uint8_t Temp) //for display return if a temp sensor is damaged or not connected 
{
  if (Temp == 255)
  {
    return true;
  }
  else
  {
    return false;
  }
}



void runPump()
{
  static bool pumpPause = false; 
  static bool pumpRun = false; 
  static uint32_t pumpTime = 0;
  static bool warm_up = false;
  static bool overheat = false;
  static uint8_t add_rpm = 0;
  static uint8_t add_tps = 0;
  uint16_t get_coldspeed;
  uint16_t get_warmspeed;
  uint8_t p_value;
  uint8_t led_value;
  
   
  uint16_t get_CLT = GETDATA(0); 

 
  if (get_CLT <= target_temp - heatup_temp & get_CLT <= target_temp - heatup_temp2 && overheat == false)
  {      
    //warm up stage 1
    // cold engine
    // pump switch between 2 states (absolutely min rpm and the value of the Ref min potentiometer)  
    // the RGB status Led shows blue     
    warm_up = true;
    after_run = false;
    overheat = false;
    warm_dis = true;
    PID = get_PWM(0,0,0,false);
    analogWrite(Out_led_g, 0); 
    digitalWrite(Out_led_r, LOW);
    digitalWrite(Out_afterrun_on, LOW);
    set_auxspeed(aux_low, 3);
    
    if (pumpTime == 0) 
    {
      pumpTime = millis();      
    } 
    
    if (pumpRun == false & ((millis() - pumpTime) <= lowspeed_time)) 
    {
      //lowspeed
      set_pumpspeed(lowspeed);
      analogWrite(Out_led_b, led_b_min);      
    }
    else if (pumpRun == false & ((millis() - pumpTime) > lowspeed_time)) 
    {
      //switch to highspeed
      pumpTime = 0; 
      pumpRun = true;
      pumpTime = millis();
    }  
    else if (pumpRun == true & ((millis() - pumpTime) <= highspeed_time))
    {
      //highspeed
      get_coldspeed = get_pot(In_low_ref, lowspeed, hot_speed);
      set_pumpspeed(get_coldspeed);
      analogWrite(Out_led_b, led_b_max);             
    } 
    else if (pumpRun == true & ((millis() - pumpTime) > highspeed_time))
    {
      //switch to lowspeed
      pumpTime = 0;
      pumpRun = false;
      pumpTime = millis();
    }          
     
  }
  else if (get_CLT < target_temp - heatup_temp2 & get_CLT > target_temp - heatup_temp)
  {
    //warm up stage 2
    //engine is bit warmer
    //pump runs on the value of the Ref min potentiometer
    //the RGB status Led shows turkis
    analogWrite(Out_led_b, led_b_min);
    analogWrite(Out_led_g, led_g_s2);
    digitalWrite(Out_led_r, LOW);
    digitalWrite(Out_afterrun_on, LOW);
    warm_up = true;
    overheat = false;
    warm_dis = false;
    after_run = false;
    PID = get_PWM(0,0,0,false);
    get_coldspeed = get_pot(In_low_ref, lowspeed, hot_speed);
    set_pumpspeed(get_coldspeed);
    set_auxspeed(aux_norm, 3);
  }  

  else if (get_CLT >= target_temp - heatup_temp2 & get_CLT < target_temp + over_temp)  
  {
    // normal operation  
    // pump runs depending on engine coolant temperature (min and max speeds are given by the ref min and ref max potentionmeter)
    // the RGB staus Led shows the current duty cycle of this mode
    warm_up = false;
    overheat = false;
    warm_dis = false;
    after_run = true;
    add_rpm = 0; //GETDATA(1);
    add_tps = 0; //GETDATA(2);
    analogWrite(Out_led_b, 0); 
    digitalWrite(Out_led_r, LOW);
    digitalWrite(Out_afterrun_on, HIGH); 
    get_coldspeed = get_pot(In_low_ref, lowspeed, hot_speed);
    get_warmspeed = get_pot(In_high_ref, get_coldspeed, max_speed);
    //p_value = map(get_CLT, 63, 90, 50, 200);
    p_value = get_PWM(get_CLT, get_coldspeed, get_warmspeed, true);
    //p_value = constrain(map(get_CLT, target_temp - heatup_temp2, target_temp, get_coldspeed, get_warmspeed), lowspeed, get_warmspeed);
    
    // for future add speed at some conditions of driving
    if (add_rpm > rpm_limit)
    {
      p_value = p_value + add_rpm_v;
    }
    if (add_tps > tps_limit) 
    {
      p_value = p_value + add_tps_v;
    }

    set_pumpspeed(p_value);
    set_auxspeed(aux_norm, 3);
    led_value = map(p_value, no_speed, get_warmspeed, led_g_min, led_g_max);
    analogWrite(Out_led_g, led_value);
      
  } 
       
  else //if (get_CLT >= target_temp + over_temp)
  // engine coolant is overheating
  // pump running full speed (Limp mode)
  // status RGB Led shows red   
  {
    set_pumpspeed(hot_speed);
    set_auxspeed(aux_hot, 3);
    overheat = true;
    after_run = true;
    warm_up = false;
    warm_dis = false;
    analogWrite(Out_led_b, 0);
    analogWrite(Out_led_g, 0);
    digitalWrite(Out_led_r, HIGH);
    digitalWrite(Out_afterrun_on, HIGH);

  }


}
void Fan () // function to control the electrical coolant fan
{
  get_CLT = GETDATA(0);
  if (get_CLT >= target_temp + start_fan_diff_temp && fan_on == false)
  {
    fan_on = true;
    digitalWrite(Out_fan, HIGH);         
  } 
  else if (get_CLT <= target_temp + stop_fan_diff_temp && fan_on == true)
  {
    fan_on = false;
    digitalWrite(Out_fan, LOW);    
  } 
      
}

bool afterrun_ok() // get the state if the after run mode is needed
{
  if (digitalRead (In_run) == LOW && after_run == true)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

void afterrun_condition (bool (condition)(void))
// function of the afterrun mode
// if the afterrun mode is enabled the pump runs an additional time when the engine stopped
// to prevent temperature hot spots in the turbocharger or engine 
// status RGB led blinking green and blue
// the display shows the countdown till completly shut down
// by pressing the push button the controller shutdown immediately 
{
  uint16_t cnt1 = 0;
  uint16_t cnt2 = 0;
  uint8_t cnt3 = 0;
  uint8_t led_b = 0;
  uint8_t led_g = 30;

   
  while (condition())    
  {
    get_CLT = GETDATA(0);
    analogWrite(Out_led_b, led_b);
    analogWrite(Out_led_g, led_g);
    cnt1 = cnt1 + 1;
    cnt2 = cnt2 + 1;
    if (get_CLT > target_temp + stop_fan_diff_temp)
    // till engine is hot pump runs at high speed an coolant fan is running 
    {
      digitalWrite(Out_fan, HIGH);
      set_pumpspeed(hot_speed);
      set_auxspeed(aux_hot, 3);
      cnt3 = 0;                
    } 
    else
    // engine isn't hot. pump runs at a slower speed
    {
      if (digitalRead(Out_fan) && cnt3 > 30)
      {
        digitalWrite(Out_fan, LOW);
        cnt3 = 0;
      }
      cnt3 = cnt3 + 1;
      set_pumpspeed(midspeed);
      set_auxspeed(aux_norm, 3);
    } 
    if (cnt1 > afterrun_time)
    {
      // shutdown everthing
      analogWrite(Out_pump_pwm, 0);
      analogWrite(Out_aux_pwm, 0);
      digitalWrite(Out_fan, LOW);
      analogWrite(Out_led_b, 0);
      digitalWrite(Out_led_r, LOW);
      analogWrite(Out_led_g, 0);
      digitalWrite(Out_afterrun_on, LOW);      
      
    }
    if (cnt2 > blink_time)
    {
      if (led_b == 0)
      {
        led_b = 20;
        led_g = 0;
        cnt2 = 0;                
      }  
      else
      {
        led_b = 0;
        led_g = 30;
        cnt2 = 0;  
      }        
    }

    if (digitalRead(12) == true || get_CLT < target_temp - heatup_temp2) // button pressed to or temp low... abort
    {
      cnt1 = afterrun_time;
    }
  
    Afterrun_DISP(F("Time Till Shutdown"),afterrun_time - cnt1, 0, afterrun_time);
    Serial.print("Time: ");
    Serial.println(cnt3);
    Serial.print("CLT: ");
    Serial.print(get_CLT);
    Serial.println(" C°");
    
    delay(100);    
  }
}

void afterrun_control () // afterrun control function 
{
  afterrun_condition(afterrun_ok);
}

void set_pumpspeed (uint8_t pumpspeed) 
// set the pump speed to PWM
{ 
  pumpspeed = constrain(pumpspeed, no_speed, max_speed);  // limit max PWM output to 34 bis 220.
  analogWrite(Out_pump_pwm, 255 - pumpspeed); //PWM value negated
  pump_duty = map(pumpspeed, no_speed, max_speed, 0, 100);  // Value for display
  //Serial.print("Pumpspeed: ");
  //Serial.print(pump_duty);
  //Serial.println("%");
}

void set_auxspeed (uint8_t auxspeed, uint8_t time_factor) 
// set the AUX pump speed to PWM
{ 
  static int16_t auxspeed_2 = 0;
  //Serial.print("SOLL Speed: ");
  //Serial.println(auxspeed);
  //Serial.print("IST Speed:");
  //Serial.println(auxspeed_2);

  if (auxspeed > auxspeed_2) //soft start
    {
      auxspeed_2 = auxspeed_2 + time_factor;
      if (auxspeed_2 > 255)
      {
        auxspeed_2 = 255;
      }
    }
  else if (auxspeed < auxspeed_2)
    {
      auxspeed_2 = auxspeed_2 - time_factor;
      if (auxspeed_2 < 0)
      {
        auxspeed_2 = 0;
      }
    }
   
  
  auxspeed_2 = constrain(auxspeed_2, 0, 255);  // limit max PWM output
  analogWrite(Out_aux_pwm, auxspeed_2); //PWM value
  aux_duty = map(auxspeed_2, 0, 255, 0, 100);  // Value for display 
  //Serial.println(auxspeed_2);
  //Serial.println(auxspeed);
}
  
uint16_t GETDATA (uint8_t x)
// get needed sensor data
// in future I will get this data from the ecu directly 
{
  // x Data from ECU
  // 0 = CLT
  // 1 = RPM
  // 2 = TPS 
  
  uint16_t data;

  if (x == 0)
  {
    data = getCLTTemp(); //getByte(7) - 40, 0, 120, 0; 
    return data; 
  }
  else if (x == 1)
  {
    data = 0; //getWord(14), 0, 9999;
    return data; 
  }
  else if (x == 2)
  {
    data = 0; //getWord(73), 0, 255;
    return data; 
  }
  else 
  {
    data = 0;
    return data;   
  } 


}

int16_t get_PWM (uint16_t clt, uint16_t low_point, uint16_t high_point, bool run)
{
  int8_t t_error;
  int16_t P;
  static int16_t D = 0;
  //static int16_t PID;
  uint16_t PID2;
  static int8_t t_error_old = 0;
  static int8_t t_error_old2 = 0;
  static int16_t I = 0;
  static uint8_t i_cnt = 0;
  static uint8_t d_cnt = 0;


  if (run == false)
  {
    t_error_old = 0;
    I = 0;
    i_cnt = 0;
    d_cnt = 0;
    PID = -1000;
  }
  else
  {


    // P part calculation
    t_error = target_temp - clt;
    P = t_error * kp;
    // time loop
    i_cnt = i_cnt + 1;
    d_cnt = d_cnt + 1;

    //I part calculation
    if (i_cnt > i_loops)
    {
      if (PID > -1000 && PID < 0)
      {
        I += t_error * ki;
        i_cnt = 0;
      }
      else
      {
        I = constrain(I, -1000, 0);
      }

    }

    //D part calculation
    if (d_cnt > d_loops)
    {
      if (t_error - t_error_old2 < 0)
      {
        D = (t_error - t_error_old2) * kd;

      }
      else
      {
        D = 0;
      }
      t_error_old2 = t_error;
      d_cnt = 0;
      
    }

    t_error_old = t_error;

    //PID calulation
    PID = P + I + D;  // + I + D;
    PID = constrain(PID, -1000, 0);
    PID2 = map(PID, -1000, 0, low_point, high_point);


  }  
  return PID2;
}

void testmode_init()
{
  //shutdown everthing
  analogWrite(Out_pump_pwm, 0);
  analogWrite(Out_aux_pwm, 0);
  analogWrite(Out_led_b, 0);
  digitalWrite(Out_led_r, LOW);
  analogWrite(Out_led_g, 0);

  //turn on Fan
  digitalWrite(Out_fan, HIGH);

  //turn off afterrun function
  digitalWrite(Out_afterrun_on, LOW); 
}

void testmode() //Pump tests
{
  uint16_t get_pumpspeed = get_pot(In_high_ref, 0, 255);
  uint16_t get_time = get_pot(In_low_ref, 0, 10);
  set_auxspeed(constrain(get_pumpspeed, aux_min, aux_hot), get_time);
  set_pumpspeed(map(get_pumpspeed, 0, 255, no_speed, max_speed));
  digitalWrite(Out_led_r, HIGH);
  digitalWrite(Out_led_b, HIGH);
  Serial.print("Time: ");
  Serial.println(get_time);
}
