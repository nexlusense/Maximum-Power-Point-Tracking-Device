#include "MPPT.h"
#include "HardwareTimer.h"

static MPPT_HandleTypeDef *hinst_mppt = NULL;

/*
* Initialize MPPT Instance & Start Interrupt Process
* Input Params - MPPT_HandleTypeDef *pmppt
*                uint16_t sample_rate
*                double panel_vsense_r
*                double batteryload_vsense_r
*                double vmax
*                double imax
* Returns      - None
*/
void MPPTCC_Init(MPPT_HandleTypeDef *pmppt, uint16_t sample_rate, double panel_vsense_r, double batteryload_vsense_r, double vmax, double imax)
{
  // Configure voltage divider ratios for voltage and current sensing circuits.
  pmppt->panel_vsense_ratio = panel_vsense_r;
  pmppt->batteryload_vsense_ratio = batteryload_vsense_r;

  // Clamp sample rate to MPPT_SENSE_SAMPLE_RATE_MAX
  if (sample_rate > MPPT_SENSE_SAMPLE_RATE_MAX)
  {
    sample_rate = MPPT_SENSE_SAMPLE_RATE_MAX;
  }

  // Initialization
  pmppt->mppt_sense_sample_rate = sample_rate;

  pmppt->battery_vmax = vmax;
  pmppt->battery_imax = imax;

  pmppt->curr_panel_v = 0.0F;
  pmppt->prev_panel_v = 0.0F;
  pmppt->curr_panel_i = 0.0F;
  pmppt->prev_panel_i = 0.0F;
  pmppt->curr_panel_p = 0.0F;
  pmppt->prev_panel_p = 0.0F;

  pmppt->curr_batload_v = 0.0F;
  pmppt->curr_batload_i = 0.0F;

  pmppt->curr_load_i = 0.0F;

  pmppt->curr_bat_i = 0.0F;

  pmppt->busy = 0U;
  pmppt->inhibit_charging = 0U;

  pmppt->adc_cal_value = 0.0F;

  pmppt->pwm_set = 0U;

  MPPTCC_PWMInit();
  MPPTCC_ADCInit(pmppt);
  MPPTCC_InterruptInit(pmppt);
}

// Pause MPPT Charge Controller Process
// Input Params - MPPT_HandleTypeDef *pmppt
// Returns      - None
//
void MPPTCC_PauseCharging(MPPT_HandleTypeDef *pmppt)
{
  pmppt->inhibit_charging = 1U;
}

// Resume MPPT Charge Controller Process
// Input Params - MPPT_HandleTypeDef *pmppt
// Returns      - None
//
void MPPTCC_ResumeCharging(MPPT_HandleTypeDef *pmppt)
{
  pmppt->inhibit_charging = 0U;
}

/*
* Get Current Charge Information
* Input Params - MPPT_HandleTypeDef *pmppt
*                 double *panel_v
*                 double *panel_i
*                 double *bat_v
*                 double *bat_i
*                 double *load_v
*                 double *load_i
*  Returns      - MPPT_StatusTypeDef  
*/
MPPT_StatusTypeDef MPPTCC_GetPowerInfo(MPPT_HandleTypeDef *pmppt, double *panel_v, double *panel_i, double *bat_v, double *bat_i, double *load_v, double *load_i)
{
  // If the MPPT algorithm is not busy computing, collect the present measurements. 
  if (!(pmppt->busy))
  {
    *panel_v = pmppt->curr_panel_v;
    *panel_i = pmppt->curr_panel_i;
    *bat_v = pmppt->curr_batload_v;
    *bat_i = pmppt->curr_bat_i;
    *load_v = pmppt->curr_batload_v;
    *load_i = pmppt->curr_load_i;

    return MPPTCC_OK;
  }
  else
  {
    return MPPTCC_BUSY;
  }
}

/*
* Initialize ADC & Calibrate
* Input Params - MPPT_HandleTypeDef *pmppt
* Returns      - None
*/
static void MPPTCC_ADCInit(MPPT_HandleTypeDef *pmppt)
{
  // Set the bit resolution of analogRead to 12 bits. 
  analogReadResolution(12U);
  MPPTCC_ADCCalibrate(pmppt);
}

/*
* Calibrate ADC
* Input Params - MPPT_HandleTypeDef *pmppt
* Returns      - None
*/
static void MPPTCC_ADCCalibrate(MPPT_HandleTypeDef *pmppt)
{
  // Compute a correction factor between the reference voltage and the measured voltage. 
  pmppt->adc_cal_value = VREF_INT / ((double)analogRead(AVREF) * ADC_FS_V / ADC_FS_COUNT);
}

/*
* Initialize MPPT Charge Controller ISR
* Input Params - MPPT_HandleTypeDef *pmppt
* Returns      - None
*/
static void MPPTCC_InterruptInit(MPPT_HandleTypeDef *pmppt)
{
  // Link instance for wrapper.
  hinst_mppt = pmppt;

  HardwareTimer *htimint = new HardwareTimer(TIM1);

  // Pause the hardware timer counter and output channels.
  htimint->pause();

  // Configure the hardware timer so that an interrupt function occurs every 0.5 ms (2000 Hz).   
  htimint->setOverflow(pmppt->mppt_sense_sample_rate, HERTZ_FORMAT);

  // Configure the hardware timer so that the function that samples the voltage and current measurements
  // as well as computes power information is called every 0.5 ms (2000 Hz).
  htimint->attachInterrupt(MPPTCC_SenseCallbackWrapper);

  // Update all hardware registers just in case any changes occurred during initialization. 
  htimint->refresh();

  // Resume the hardware timer counter and output channels.
  htimint->resume();
}

/*
* ISR Wrapper
* Input Params - None
* Returns      - None
*/
static void MPPTCC_SenseCallbackWrapper(void)
{
  MPPTCC_SenseCallback(hinst_mppt);
}

/*
* Callback For Sampling & Calculating Power Information & PWM Process
* Input Params - MPPT_HandleTypeDef *pmppt
* Returns      - None
*
*/
static void MPPTCC_SenseCallback(MPPT_HandleTypeDef *pmppt)
{
  // Indicate that the MPPT algorithm is busy computing. 
  pmppt->busy = 1;

  // If the MPPT controller circuit is not currently charging the battery, measure the voltage and current. 
  if (!(pmppt->inhibit_charging))
  {
    // Measure the voltage and current from the solar panel and compute its power. 
    pmppt->curr_panel_v = ((double)analogRead(PANEL_VSENSE) * ADC_FS_V * pmppt->adc_cal_value / ADC_FS_COUNT) * pmppt->panel_vsense_ratio;
    pmppt->curr_panel_i = (double)analogRead(PANEL_ISENSE) * ADC_FS_V * pmppt->adc_cal_value / ADC_FS_COUNT;
    pmppt->curr_panel_p = pmppt->curr_panel_v * pmppt->curr_panel_i;

    // Measure the voltage and current from the battery load.
    pmppt->curr_batload_v = ((double)analogRead(BATTERYLOAD_VSENSE) * ADC_FS_V * pmppt->adc_cal_value / ADC_FS_COUNT) * pmppt->batteryload_vsense_ratio;
    pmppt->curr_batload_i = (double)analogRead(BATTERYLOAD_ISENSE) * ADC_FS_V * pmppt->adc_cal_value / ADC_FS_COUNT;

    // Measure the current from the non-battery load. 
    pmppt->curr_load_i = (double)analogRead(LOAD_ISENSE) * ADC_FS_V * pmppt->adc_cal_value / ADC_FS_COUNT;

    // Compute the net current of the battery. 
    pmppt->curr_bat_i = pmppt->curr_batload_i - pmppt->curr_load_i;


    MPPTCC_ControlPWM(pmppt);
    MPPTCC_PWMWrite(pmppt->pwm_set);

    // Set the currently measured values as past measurements.
    pmppt->prev_panel_v = pmppt->curr_panel_v;
    pmppt->prev_panel_i = pmppt->curr_panel_i;
    pmppt->prev_panel_p = pmppt->curr_panel_p;
  }
  else
  {
    MPPTCC_PWMWrite(0U);
  }

  // MPPT Algorithm is no longer busy computing. 
  pmppt->busy = 0;
}

/*
* MPPT Charge Control Algorithm
* Input Params - MPPT_HandleTypeDef *pmppt
* Returns      - None
*/
static void MPPTCC_ControlPWM(MPPT_HandleTypeDef *pmppt)
{
  if (pmppt->curr_bat_i > pmppt->battery_imax)
  {
    // Overcurrent | REDUCE Power
    pmppt->pwm_set--;
  }
  else if (pmppt->curr_batload_v > pmppt->battery_vmax)
  {
    // Overvoltage | REDUCE Power
    pmppt->pwm_set--;
  }
  else
  {
    // Output V & I OK | CONTINUE to Power Point Tracking
    if (pmppt->curr_panel_p > pmppt->prev_panel_p && pmppt->curr_panel_v > pmppt->prev_panel_v)
    {
      // Increase in P & Increase in V 
      // PV-curve point is moving right and towards MPP | REDUCE Power
      pmppt->pwm_set--;
    }
    else if (pmppt->curr_panel_p < pmppt->prev_panel_p && pmppt->curr_panel_v > pmppt->prev_panel_v)
    {
      // Decrease in P & Increase in V 
      // PV-curve point is moving right and away from MPP | INCREASE Power
      pmppt->pwm_set++;
    }
    else if (pmppt->curr_panel_p > pmppt->prev_panel_p && pmppt->curr_panel_v < pmppt->prev_panel_v)
    {
      // Increase in P & Decrease in V 
      // PV-curve point is moving left and towards MPP | INCREASE Power
      pmppt->pwm_set++;
    }
    else if (pmppt->curr_panel_p < pmppt->prev_panel_p && pmppt->curr_panel_v < pmppt->prev_panel_v)
    {
      // Decrease in P & Decrease in V
      // PV-curve point is moving left and away from MPP | REDUCE Power
      pmppt->pwm_set--;
    }
    else
    {
      // MAINTAIN Power Setting
      pmppt->pwm_set = pmppt->pwm_set;
    }
  }

}

/*
* Initialize PWM Subsystem
* Input Params - None
* Returns      - None
*/
static void MPPTCC_PWMInit(void)
{
  // Set PWM signal frequency to 150k Hz.
  analogWriteFrequency(150000U);

  // Set the bit resolution of the analog write function to 16 bits.
  analogWriteResolution(16U);

  // Initialize the PWM signal for the MOSFET gate driver with a duty cycle of 0%.
  analogWrite(GATE_DRIVER_IN, 0U);
}

/*
* Write Value to PWM Subsystem
* Input Params - uint16_t duty
* Returns      - None
*/
static void MPPTCC_PWMWrite(uint16_t duty)
{
  if (duty == 0)
  {
    // Shutdown the MOSFET gate driver if the duty cycle is 0%. 
    digitalWrite(GATE_DRIVER_SD, LOW);
  }
  else
  {
    // Disable the shutdown feature of the MOSFET gate driver if the duty cycle is greater than 0%. 
    digitalWrite(GATE_DRIVER_SD, HIGH);
  }

  // Send the PWM signal to the input of the MOSFET gate driver. 
  analogWrite(GATE_DRIVER_IN, duty);
}
