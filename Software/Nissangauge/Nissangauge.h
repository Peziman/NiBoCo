#define SERIAL_UPDATE_MS  100
#define N_PAGES 6
#define Btn_next 2 //Button 1 (Next)
#define Btn_prev 4 //Button 2 (Prev)
#define Btn_push 12 //Button 3 (Enter)
#define BATT_RAW A6 //Input Pin for car battery voltage

#define REF_VOLTAGE 5.0 //Reference voltage for car battery voltage calculations
#define PIN_STEPS 1024.0 ////Reference steps for car battery voltage calculations

#define BUTTON_HOLD_TIME 3000

/* ----- Engine status ----- */ //FOR LATER USE
#define ENGINE_OFF		0 
#define ENGINE_WARMUP	1
#define ENGINE_RUN		2

#define IDLE_RPM_ENTER  1000
#define IDLE_RPM_EXIT   1100
#define IDLE_DELAY_MS   2000
/* ----- Engine status end ----- */

uint16_t get_VOLT();
