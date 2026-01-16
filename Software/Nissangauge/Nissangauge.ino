#include "Arduino.h"
#include "Pages.h"
#include "Pump_control.h"
#include "Nissangauge.h"

const float diodeVoltage = 0.538;  // forward voltage of the used diode in Volts
const float R1 = 98500.0;          // exact resistance of R1 (= 100 kOhm)
const float R2 = 9410.0;           // exact resistance of R2 (= 10 kOhm)

uint8_t pageNum = 0;
bool test_mode = false;
uint8_t ticks = 0;

void setup() {
  Serial.begin(9500);
  initDisplay();
  if (pageNum >= N_PAGES) {
    pageNum = 0;
  }
  initPump();
  pinMode(Btn_next, INPUT);
  pinMode(Btn_prev, INPUT);
  pinMode(Btn_push, INPUT);
  pinMode(BATT_RAW, INPUT);
}

void tik_tak()
{
  if (ticks > 255)
  {
    ticks = 0;
  }
  else
  {
    ticks++;
  }
}


uint16_t get_VOLT()
{
  float vout = 0.0;
  float BATT_V = 0.0;
  uint16_t BATT_V2 = 0;
  uint16_t rawValue = 0;
  rawValue = analogRead(BATT_RAW);
  vout = (rawValue * REF_VOLTAGE) / PIN_STEPS;
  BATT_V = vout / (R2 / (R1 + R2)) * 10;
  BATT_V = BATT_V + diodeVoltage;
  BATT_V2 = (uint16_t)BATT_V;
  return BATT_V2;
}


void fail_check()
{
  if (get_overtemp() == true && ticks == 1)
  {
    pageNum = 11;
  }
  if (getOilTemp() > 135 && ticks == 100)
  {
    pageNum = 12;
  }
  if (get_VOLT() > 1500 || get_VOLT() < 110 && ticks == 200)
  {
    pageNum = 13;
  }
  //Serial.println(get_VOLT());
}

void loop() {

  // button and page number operations
  static bool buttonLast = false;
  static bool buttonHeld = false;
  static uint32_t timePressed = 0;
  //static uint8_t oilTemp = 0;
  //static uint8_t cltTemp = 0;


  static bool btn_nextNow = digitalRead(Btn_next);
  static bool btn_prevNow = digitalRead(Btn_prev);
  static bool btn_pushNow = digitalRead(Btn_push);
  static bool btn_nextHIGH = false;
  static bool btn_prevHIGH = false;
  static bool btn_pushHIGH = false;
  //digitalWrite(LED_BUILTIN, btn_nextNow);



  //button Next pressed

  if (digitalRead(Btn_next) == true) 
  {
    btn_nextHIGH = true;
  }
  if (digitalRead(Btn_next) == false && btn_nextHIGH == true) 
  {
    if (test_mode == false)
    {
      pageNum++;
      if (pageNum >= N_PAGES)
        pageNum = 0;
      btn_nextHIGH = false;
    }
  }

  //button Prev pressed

  if (digitalRead(Btn_prev) == true)  
  {
    btn_prevHIGH = true;
  }
  if (digitalRead(Btn_prev) == false && btn_prevHIGH == true)
  {
    if (test_mode == false)
    {
      pageNum--;
      if (pageNum >= N_PAGES)
        pageNum = N_PAGES - 1;   
      btn_prevHIGH = false;
    }
  }


  //button Enter pressed

  if (digitalRead(Btn_push) == true && btn_pushHIGH == false)
  {
    btn_pushHIGH = true;
    timePressed = millis();
  }
  
  if (digitalRead(Btn_push) == false && btn_pushHIGH == true) 
  {
    Serial.println(millis() - timePressed);
    if ((millis() - timePressed) > BUTTON_HOLD_TIME)
    {
      if (test_mode == false)
      {
        test_mode = true;
        testmode_init();
        pageNum = 10;
      }
      else
      {
        test_mode = false;
        pageNum = 0;
      }
    }
    else
    {
      if (test_mode == false)
      {
        pageNum++;
        if (pageNum >= N_PAGES)
          pageNum = 0;
      }
    }
    btn_pushHIGH = false;
  }

  // serial operation, frequency based request
  static uint32_t lastUpdate = millis();
  if (millis() - lastUpdate > SERIAL_UPDATE_MS) {
    lastUpdate = millis();
    if (test_mode == true)
    {
      testmode();
    }
    else
    {
      runPump();
      Fan();
      afterrun_control();
      tik_tak();
      fail_check();
    }
    //oilTemp = getOilTemp();
    //cltTemp = getCLTTemp();

  }

  // get refresh rate
  static uint32_t lastRefresh = millis();
  uint16_t refreshRate = 1000 / (millis() - lastRefresh);
  lastRefresh = millis();

  // display pages
  switch (pageNum) {
    case 0:
      show2Bar(F("Water Temperatur (\367C)"), getCLTTemp(), 0, 120, 0,
               F("Oil Temperatur (\367C)"), getOilTemp(), 0, 150, 0);
      //showBar(F("Engine Speed (RPM)"), getWord(14), 0, 6000);
      break;
    case 1:
      showBar(F("Battery (V)"), get_VOLT(), 60, 160, 1);
      break;
    case 2:
      show4Numeric(F("Pumpspd %"), get_pump_duty(), 0, 100, 0,
                   F("CLT Temp\367C"), getCLTTemp(), 0, 120, 0,
                   F("Min Speed%"), get_pot_duty(0), 0, 100, 0,
                   F("Max Speed%"), get_pot_duty(1), 0, 100, 0);
      break;
    case 3:
      showFlags(F("Fan"), get_fan_state(),
                F("RUN"), get_ign_state(),
                F("AFTERRUN"), get_afterrun(),
                F("warm"), get_warmup(),
                F("CLT ERROR"), get_Temp_sens_state(getCLTTemp()),
                F("OIL Error"), get_Temp_sens_state(getOilTemp()),
                F(""), false,
                F(""), false);
      //showBar(F("Coolant Temp (\367C)"), (int16_t)getByte(7) - 40, 0, 120);
      break;
    case 4:
      show2Bar(F("AUX Pumpspeed %"), get_aux_duty(), 0, 100, 0,
               F("Water Temperatur (\367C)"), getCLTTemp(), 0, 120, 0);
      break;
    case 5:
      showNumeric(F("FPS"), refreshRate, 0, 100, 0);
      break;  
    case 10:
      show4Numeric(F("Pumpspd %"), get_pump_duty(), 0, 100, 0,
                   F("AUX Pump %"), get_aux_duty(), 0, 100, 0,
                   F("Lower Pot %"), test_pot_duty(0), 0, 100, 0,
                   F("Top Pot %"), test_pot_duty(1), 0, 100, 0);
      break;
    case 11:
      showBar(F("CLT Temp\367C FAIL"), getCLTTemp(), 0, 120, 0);
      break;  
    case 12:
      showBar(F("OIL Temp\367C FAIL"), getOilTemp(), 0, 150, 0);
      break;
    case 13:
      showBar(F("Battery (V) FAIL"), get_VOLT(), 60, 160, 1);
      break;      
    default:
      showSplash(F("Coming Soon!"));
      break;
  }
}
