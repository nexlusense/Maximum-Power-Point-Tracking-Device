#include <stdint.h>
#include <SoftwareSerial.h>

#define MFN_RX_PIN 2
#define MFN_TX_PIN 3
#define MFN_REQUEST_TIMEOUT_MS 500

#define MAX_DEVICE_COMMANDS 256
#define DEVICEID_POWER_COMMAND__BATTERY_LEVEL 0x00
#define DEVICEID_POWER_COMMAND__BATTERY_HEALTH 0x01
// Add more COMMANDs here

typedef enum
{
  MFN_OK = 0x00,
  MFN_TIMEOUT_OCCURED = 0x01,
  MFN_DATA_UNAVAILABLE = 0x02,
  MFN_NOT_TARGET = 0x03,
  MFN_BAD_TARGET_REPLY = 0x04,
  MFN_ERROR = 0x05
} MFN_StatusTypeDef;

typedef enum
{
  MFN_WRITE_ACCESS = 0x00,
  MFN_READ_ACCESS = 0x01,

  // Device IDs
  MFN_DEVICEID_NONE = 0xFF,
  MFN_DEVICEID_POWER = 0x00,
  MFN_DEVICEID_SENSOR_ADDON = 0x01,
  // Add more DEVICEIDs here

  // Enumerators
  MFN_DEVICE_0 = 0x00,
  MFN_DEVICE_1 = 0x01,
  MFN_DEVICE_2 = 0x02,
  MFN_DEVICE_3 = 0x03,
  MFN_DEVICE_4 = 0x04,
  MFN_DEVICE_5 = 0x05,
  MFN_DEVICE_6 = 0x06,
  MFN_DEVICE_7 = 0x07

} MFN_DeviceTypeDef;

typedef struct
{
  uint8_t device_data[MAX_DEVICE_COMMANDS];
  uint8_t is_host;
  MFN_DeviceTypeDef device_id;
  MFN_DeviceTypeDef device_enum;
} MFN_HandleTypeDef;

MFN_StatusTypeDef MFN_Init(MFN_HandleTypeDef *hmfn);
MFN_StatusTypeDef MFN_SetAsHost(MFN_HandleTypeDef *hmfn);
MFN_StatusTypeDef MFN_SetAsDevice(MFN_HandleTypeDef *hmfn, MFN_DeviceTypeDef device, MFN_DeviceTypeDef enumerator);
MFN_StatusTypeDef MFN_SendRequest(MFN_HandleTypeDef *hmfn, MFN_DeviceTypeDef device_id, MFN_DeviceTypeDef enumerator, uint8_t command, uint8_t *pdata);
MFN_StatusTypeDef MFN_AnswerRequestOnAvailable(MFN_HandleTypeDef *hmfn);
MFN_StatusTypeDef MFN_DeviceAccessData(MFN_HandleTypeDef *hmfn, MFN_DeviceTypeDef rw, uint8_t command, uint8_t *pdata);



