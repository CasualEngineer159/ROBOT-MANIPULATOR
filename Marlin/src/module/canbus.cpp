#include "canbus.h"
#include "../core/serial.h"

#if ENABLED(USE_CANBUS)

// Removed the static pin checks since we are hardcoding PD0/PD1 for the BTT Octopus Pro
// #if !PIN_EXISTS(CAN_RX) || !PIN_EXISTS(CAN_TX)
//   #error "CANBUS requires CAN_RX_PIN and CAN_TX_PIN."
// #endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

CAN_HandleTypeDef hcan1;

CANBus canbus;

CANBus::CANBus() {}

void CANBus::setup_canbus() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE(); // Enable GPIOD for PD0 and PD1

  // CAN_TX = PD1 (Push-Pull, No Pull)
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // CAN_RX = PD0 (Push-Pull hardware overridden to Input, Add Pull-Up)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // <-- Added Pull-Up for stability
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 15;
  hcan1.Init.Mode = CAN_MODE_NORMAL;

  // Zvýšení tolerance pro resynchronizaci hodin mezi dvěma čipy (z 1TQ na 2TQ)
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;

  // Posun bodu vzorkování (Sample Point) na ~73 % ( 1 + 10 = 11; 11 / 15 = 73,3% )
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;

  // <-- CRITICAL CHANGE: Stop infinite retries on missing ACK
  hcan1.Init.AutoRetransmission = DISABLE;

  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    SERIAL_ECHOLN("CAN Init Failed");
    return;
  }

  // (Filter setup remains the same, it correctly lets everything through)
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    SERIAL_ECHOLN("CAN Filter Config Failed");
    return;
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    SERIAL_ECHOLN("CAN Start Failed");
    return;
  }
}

void CANBus::send_message(uint32_t id, uint8_t* data, uint8_t len) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  TxHeader.StdId = id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = len;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) {
    SERIAL_ECHOLN("CAN Send Failed");
  }
}

bool CANBus::receive_message(uint32_t* id, uint8_t* data, uint8_t* len) {
  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
    return false;
  }

  CAN_RxHeaderTypeDef RxHeader;
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, data) != HAL_OK) {
    SERIAL_ECHOLN("CAN Receive Failed");
    return false;
  }

  *id = RxHeader.StdId;
  *len = RxHeader.DLC;

  return true;
}

#endif // USE_CANBUS
