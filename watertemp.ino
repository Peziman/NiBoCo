#include "Arduino.h"

/* WATER TEMPERATURE SENSOR ON PIN A3

ADC values are calculated as  ADC = ADCmax / Vcc * Vsens
                      where   Vsens = Vcc / (Rsens + Rbias) * Rsens
Entries in the cltADC array are calculated with Vcc = 5V and Rbias = 2700 ohms            

*/
#define CLT_TABLE_SIZE 13 //Don't touch
#define CLTTEMP_FILTER 5 //smooth out temperature changes lower values = faster, higher values = slower
#define ADC2_NO_CONN    1000 // ADC value to detect broken sensor
#define ADC2_COLD       700 // ADC value to detect cold condition

//#define RAW_ADC

const uint16_t cltADC[CLT_TABLE_SIZE] =     {40,   53,    67,    87,    111,    143,    186,   238,  304, 395, 492, 574, 700};
const uint8_t cltDegrees[CLT_TABLE_SIZE] = {120,   110,   100,   90,   80,   70,   60,   50,  40,  30,  20,  10,  1};

uint8_t getCLTTemp()
{
  static uint16_t adcValue = 0;
  static bool cltCold = true;
  uint8_t i = 1;
  uint8_t cltLast;

  adcValue  = analogRead(A3);

  #ifdef RAW_ADC
    return adcValue / 4;
  #endif
  
  if((adcValue > ADC2_COLD)){ //cltCold && 
  //  cltCold = false; // Water is considered cold until it reaches a given temperature
    cltCold = true;
  }  
  else
  {
    cltCold = false;
  }

  if(adcValue > ADC_NO_CONN) return 255; // Sensor not connected
  if(cltCold) return 0; // Water is cold

  if(adcValue <= cltADC[0])
    cltLast = cltDegrees[0]; // Return highest value
  else
  {
    while(adcValue > cltADC[i]) i++;

    if(adcValue == cltADC[i])
      cltLast = cltDegrees[i];
    else
      cltLast = cltDegrees[i] + (cltDegrees[i-1] - cltDegrees[i]) * (cltADC[i] - adcValue) / (cltADC[i] - cltADC[i-1]);
  }

  static uint16_t cltSmooth = cltLast * CLTTEMP_FILTER;
  if(cltSmooth / CLTTEMP_FILTER < cltLast)
    cltSmooth++;
  else if(cltSmooth / CLTTEMP_FILTER > cltLast)
    cltSmooth--;

  return cltSmooth / CLTTEMP_FILTER;


}