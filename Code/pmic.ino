// Example usage of the MPPT Charge Controller
// An ARM Cortex-M3 or better is recommended

#include "MPPT.h"

// Sampling Rate of Voltage/Current Sensing
#define SENSING_RATE_HZ       2000U

// Voltage Divider Ratios
#define PANEL_VSENSE_RATIO    MPPTCC_RRATIO(8660U, 1320U) // MPPTCC_RRATIO(Rtop, Rbottom) in Ohms
#define OUTPUT_VSENSE_RATIO   MPPTCC_RRATIO(8660U, 2500U)

// Maximum voltage and current for battery load
#define BATTERY_VMAX          14.6F
#define BATTERY_IMAX          2.5F

MPPT_HandleTypeDef hmppt;

double panel_voltage = 0;
double panel_current = 0;
double panel_power = 0;

double battery_voltage = 0;
double battery_current = 0;
double battery_power = 0;

double load_voltage = 0;
double load_current = 0;
double load_power = 0;

double converter_efficiency = 0;

void setup()
{
  // Start MPPT process; everything is done via ISR
  MPPTCC_Init(&hmppt, SENSING_RATE_HZ, PANEL_VSENSE_RATIO, OUTPUT_VSENSE_RATIO, BATTERY_VMAX, BATTERY_IMAX);
  Serial.begin(115200U);
}

void loop()
{
  // Since MPPT process is handled via ISR, other code can go here

  // Print power information every 5s +/- 0.5s (if available)
  if ((MPPTCC_GetPowerInfo(&hmppt, &panel_voltage, &panel_current, &battery_voltage, &battery_current, &load_voltage, &load_current) == MPPTCC_OK) &&
      (millis() % 5000U < 500U))
  {
    panel_power = panel_voltage * panel_current;
    battery_power = battery_voltage * battery_current;
    load_power = load_voltage * load_current;
    converter_efficiency = 100.0F * (panel_power / (battery_power + load_power));


    /* Prints out something like this:

    Panel ----------------------------
       Power:   50.76W
       Voltage: 19.30V
       Current: 2.63A
    Battery --------------------------
       Power:   45.36W
       Voltage: 12.60V
       Current: 3.60A
    Load -----------------------------
       Power:   1.01W
       Voltage: 12.60V
       Current: 0.08A
    >> Converter Efficiency: 91.35%


    */
    Serial.println("Panel ----------------------------");
    Serial.print("   Power:   ");
    Serial.print(panel_power);
    Serial.println("W");
    Serial.print("   Voltage: ");
    Serial.print(panel_voltage);
    Serial.println("V");
    Serial.print("   Current: ");
    Serial.print(panel_current);
    Serial.println("A");

    Serial.println("Battery --------------------------");
    Serial.print("   Power:   ");
    Serial.print(battery_power);
    Serial.println("W");
    Serial.print("   Voltage: ");
    Serial.print(battery_voltage);
    Serial.println("V");
    Serial.print("   Current: ");
    Serial.print(battery_current);
    Serial.println("A");

    Serial.println("Load -----------------------------");
    Serial.print("   Power:   ");
    Serial.print(load_power);
    Serial.println("W");
    Serial.print("   Voltage: ");
    Serial.print(load_voltage);
    Serial.println("V");
    Serial.print("   Current: ");
    Serial.print(load_current);
    Serial.println("A");

    Serial.print(">> Converter Efficiency: ");
    Serial.print(converter_efficiency);
    Serial.println("%\n\n");

  }
  
}


