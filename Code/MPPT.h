#include <stdint.h>
#include "Arduino.h"

// -------------------------------------------
// STM32 Device Pinout
// -------------------------------------------

// LED Control Pins
#define LED_CHARGING PB0
#define LED_CHARGED PB1
#define LED_SYS_FAULT PB2

// Voltage and Current Sensing Pins
#define PANEL_VSENSE PA6
#define PANEL_ISENSE PA7
#define BATTERYLOAD_VSENSE PA8
#define BATTERYLOAD_ISENSE PA9
#define LOAD_ISENSE PA10

// MOSFET Gate Driver Control Pins
#define GATE_DRIVER_IN PA0 // Input PWM pin
#define GATE_DRIVER_SD PA1 // Shutdown Pin

// Analog-to-DC Converter Calibration Fields
#define VREF_INT 1.2F
#define ADC_FS_V 3.3F
#define ADC_FS_COUNT 4096.0F

// Sampling Rate of Voltage/Current Sensing
#define MPPT_SENSE_SAMPLE_RATE_MAX 2000U

// Helper Macro For Resistor Division Ratio
#define MPPTCC_RRATIO(_rt, _rb) 1.0F + (double)_rt / (double)_rb 

typedef enum
{
  MPPTCC_OK = 0x00,
  MPPTCC_BUSY = 0x01,
  MPPTCC_SYS_FAULT = 0x02
} MPPT_StatusTypeDef;

typedef struct
{
  double panel_vsense_ratio;
  double batteryload_vsense_ratio;

  uint16_t mppt_sense_sample_rate;

  double battery_vmax;
  double battery_imax;

  double curr_panel_v;
  double prev_panel_v;
  double curr_panel_i;
  double prev_panel_i;
  double curr_panel_p;
  double prev_panel_p;

  double curr_batload_v;
  double curr_batload_i;

  double curr_load_i;

  double curr_bat_i;

  uint8_t busy;
  uint8_t inhibit_charging;

  double adc_cal_value;

  uint16_t pwm_set;

} MPPT_HandleTypeDef;

void MPPTCC_Init(MPPT_HandleTypeDef *pmppt, uint16_t sample_rate, double panel_vsense_r, double batteryload_vsense_r, double vmax, double imax);
void MPPTCC_PauseCharging(MPPT_HandleTypeDef *pmppt);
void MPPTCC_ResumeCharging(MPPT_HandleTypeDef *pmppt);
MPPT_StatusTypeDef MPPTCC_GetPowerInfo(MPPT_HandleTypeDef *pmppt, double *panel_v, double *panel_i, double *bat_v, double *bat_i, double *load_v, double *load_i);
static void MPPTCC_ADCInit(MPPT_HandleTypeDef *pmppt);
static void MPPTCC_ADCCalibrate(MPPT_HandleTypeDef *pmppt);
static void MPPTCC_InterruptInit(MPPT_HandleTypeDef *pmppt);
static void MPPTCC_SenseCallbackWrapper(void);
static void MPPTCC_SenseCallback(MPPT_HandleTypeDef *pmppt);
static void MPPTCC_ControlPWM(MPPT_HandleTypeDef *pmppt);
static void MPPTCC_PWMInit(void);
static void MPPTCC_PWMWrite(uint16_t duty);





