// Example usage for MFN

#include "mfn.h"

#define HOST

MFN_HandleTypeDef hmfn;
uint8_t data = 20;
MFN_StatusTypeDef retval = MFN_OK;

void setup()
{
  Serial.begin(9600);

  MFN_Init(&hmfn);

  #ifdef HOST

  Serial.print("Setting as HOST... ");
  if (MFN_SetAsHost(&hmfn) == MFN_OK)
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("FAILED");
  }

  #else

  Serial.print("Setting as DEVICE... ");
  if (MFN_SetAsDevice(&hmfn, MFN_DEVICEID_POWER, MFN_DEVICE_0) == MFN_OK)
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("FAILED");
  }
  
  #endif
  
}

void loop()
{
  #ifdef HOST

  retval = MFN_SendRequest(&hmfn, MFN_DEVICEID_POWER, MFN_DEVICE_0, DEVICEID_POWER_COMMAND__BATTERY_LEVEL, &data);
  if (retval == MFN_OK)
  {
    Serial.print("Send Request: OK. Data = ");
    Serial.println(data);
  }
  else if (retval == MFN_ERROR)
  {
    Serial.println("Send Request: ERROR");
  }
  else if (retval == MFN_TIMEOUT_OCCURED)
  {
    Serial.println("Send Request: TIMEOUT");
  }
  else if (retval == MFN_BAD_TARGET_REPLY)
  {
    Serial.println("Send Request: BAD REPLY");
  }

  #else

  MFN_DeviceAccessData(&hmfn, MFN_WRITE_ACCESS, DEVICEID_POWER_COMMAND__BATTERY_LEVEL, &data);

  retval = MFN_AnswerRequestOnAvailable(&hmfn);
  if (retval == MFN_OK)
  {
    Serial.println("Answer Request: OK");
  }
  else if (retval == MFN_ERROR)
  {
    Serial.println("Answer Request: ERROR");
  }
  else if (retval == MFN_DATA_UNAVAILABLE)
  {
    Serial.println("Answer Request: UNAVAILABLE");
  }
  else if (retval == MFN_NOT_TARGET)
  {
    Serial.println("Answer Request: NOT TARGET");
  }

  #endif

  delay(1000);

}
