#include "mfn.h"
#include "Arduino.h"

SoftwareSerial smfn(MFN_RX_PIN, MFN_TX_PIN);

// Initialize MFN System
// Input Params - MFN_HandleTypeDef *hmfn
// Returns      - MFN_StatusTypeDef
//
MFN_StatusTypeDef MFN_Init(MFN_HandleTypeDef *hmfn)
{
  smfn.begin(9600);
  hmfn->device_id = MFN_DEVICEID_NONE;
  hmfn->is_host = 0;
  hmfn->device_enum = MFN_DEVICE_0;

  for (uint16_t position = 0; position < 256; position++)
  {
    hmfn->device_data[position] = 0;
  }

  return MFN_OK;
}

// Register System Controller As A HOST
// Input Params - MFN_HandleTypeDef *hmfn
// Returns      - MFN_StatusTypeDef
//
MFN_StatusTypeDef MFN_SetAsHost(MFN_HandleTypeDef *hmfn)
{
  if (hmfn->device_id == 0)
  {
    hmfn->is_host = 1;
    return MFN_OK;
  }
  else
  {
    return MFN_ERROR;
  }
}

// Register System Controller As A DEVICE
// Input Params - MFN_HandleTypeDef *hmfn
//                MFN_DeviceTypeDef device
//                MFN_DeviceTypeDef enumerator
// Returns      - MFN_StatusTypeDef
//
MFN_StatusTypeDef MFN_SetAsDevice(MFN_HandleTypeDef *hmfn, MFN_DeviceTypeDef device, MFN_DeviceTypeDef enumerator)
{
  if (hmfn->is_host != 1)
  {
    hmfn->device_id = device;
    hmfn->device_enum = enumerator;
    return MFN_OK;
  }
  else
  {
    return MFN_ERROR;
  }
}

// Send Command Request to DEVICE
// Input Params - MFN_HandleTypeDef *hmfn
//                MFN_DeviceTypeDef device
//                MFN_DeviceTypeDef enumerator
//                uint8_t command
//                uint8_t *pdata
// Returns      - MFN_StatusTypeDef
//
MFN_StatusTypeDef MFN_SendRequest(MFN_HandleTypeDef *hmfn, MFN_DeviceTypeDef device_id, MFN_DeviceTypeDef enumerator, uint8_t command, uint8_t *pdata)
{
  uint32_t mfn_data = 0;
  uint16_t timeout_counter = 0;
  uint16_t decoded_device_id = 0;
  uint8_t decoded_device_enum = 0;
  uint8_t received_command = 0;
  uint8_t data = 0;

  // Verify host
  if (hmfn->is_host != 1)
  {
    return MFN_ERROR;
  }

  // Build packet (sending command request)
  // [13b Device ID | 3b Enumerator | 8b Command | 8b Data]
  mfn_data = (device_id << 19) | (enumerator << 16) | (command << 8) | 0x00;

  // Send packet
  smfn.write(mfn_data);
  
  // Wait for a response within MFN_REQUEST_TIMEOUT_MS
  while (!smfn.available())
  {
    timeout_counter++;
    delay(1);

    if (timeout_counter >= MFN_REQUEST_TIMEOUT_MS)
    {
      return MFN_TIMEOUT_OCCURED;
    }
  }

  // Read received data
  mfn_data = smfn.read();

  // Decode received packet (now with data related to command request)
  // [13b Device ID | 3b Enumerator | 8b Command | 8b Data]
  decoded_device_id = mfn_data >> 19;
  decoded_device_enum = (mfn_data >> 16) & 0x07;
  received_command = (mfn_data >> 8) & 0xFF;
  data = mfn_data & 0xFF;

  // Verify the device reply
  if (decoded_device_id == device_id && decoded_device_enum == enumerator && received_command == command)
  {
    *pdata = data;
    return MFN_OK;
  }
  else
  {
    return MFN_BAD_TARGET_REPLY;
  }

}

// Answer Command Request From HOST With Data If Available
// Input Params - MFN_HandleTypeDef *hmfn
// Returns      - MFN_StatusTypeDef
//
MFN_StatusTypeDef MFN_AnswerRequestOnAvailable(MFN_HandleTypeDef *hmfn)
{
  uint32_t mfn_data = 0;
  uint16_t decoded_device_id = 0;
  uint8_t decoded_device_enum = 0;
  uint8_t received_command = 0;

  // Verify device
  if (hmfn->is_host != 0)
  {
    return MFN_ERROR;
  }

  // Check if data is available on the serial port
  if (smfn.available())
  {
    mfn_data = smfn.read();
  }
  else
  {
    return MFN_DATA_UNAVAILABLE;
  }

  // [13b Device ID | 3b Enumerator | 8b Command | 8b Data]
  decoded_device_id = mfn_data >> 19;
  decoded_device_enum = (mfn_data >> 16) & 0x07;
  received_command = (mfn_data >> 8) & 0xFF;

  // Check if device is the intended target
  if (decoded_device_id == hmfn->device_id && decoded_device_enum == hmfn->device_enum)
  {
    // Echo device ID, enumerator, command & append data
    // [13b Device ID | 3b Enumerator | 8b Command | 8b Data]
    mfn_data = (decoded_device_id << 19) | (decoded_device_enum << 16) | (received_command << 8) | hmfn->device_data[received_command];

    // Send data back
    smfn.write(mfn_data);
    return MFN_OK;

  }
  else
  {
    return MFN_NOT_TARGET;
  }

}

// Read/Write MFN Buffer Data
// Input Params - MFN_HandleTypeDef *hmfn
//                MFN_DeviceTypeDef rw
//                uint8_t command
//                uint8_t *pdata
// Returns      - MFN_StatusTypeDef
//
MFN_StatusTypeDef MFN_DeviceAccessData(MFN_HandleTypeDef *hmfn, MFN_DeviceTypeDef rw, uint8_t command, uint8_t *pdata)
{
  // Verify device
  if (hmfn->is_host != 0)
  {
    return MFN_ERROR;
  }

  // Check Read/Write Access
  if (rw == MFN_READ_ACCESS)
  {
    *pdata = hmfn->device_data[command];
    return MFN_OK;
  }
  else if (rw == MFN_WRITE_ACCESS)
  {
    hmfn->device_data[command] = *pdata;
    return MFN_OK;
  }
  else
  {
    return MFN_ERROR;
  }


}




