/**
  ******************************************************************************
  * @file    usbd_customhid_if_template.c
  * @author  MCD Application Team
  * @brief   USB Device Custom HID interface file.
  *        This template should be copied to the user folder, renamed and customized
  *          following user needs.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "../Inc/usbd_customhid_if.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static int8_t TEMPLATE_CUSTOM_HID_Init(void);
static int8_t TEMPLATE_CUSTOM_HID_DeInit(void);
static int8_t TEMPLATE_CUSTOM_HID_OutEvent(uint8_t state);

#ifdef USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED
static int8_t TEMPLATE_CUSTOM_HID_CtrlReqComplete(uint8_t request, uint16_t wLength);
#endif /* USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED */

#ifdef USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED
static uint8_t *TEMPLATE_CUSTOM_HID_GetReport(uint16_t *ReportLength);
#endif /* USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED */
/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef USBD_Device;
//To extern the report_buffer variable
extern uint8_t report_buffer[64];
extern uint8_t flag_rx;

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
		/* USER CODE BEGIN 0 */
		0x05, 0x01, 0xff, // Usage Page (Generic Desktop)
		0x09, 0x05, // USAGE (Game Pad)
		0xa1, 0x01, // COLLECTION (Application)
		0x05, 0x09, // USAGE (Button)
		0x19, 0x01,  // USAGE MINIMUM (1)
		0x29, 0x09,  // USAGE MAXIMUM (9)
		0x15, 0x01, // LOGICAL_MINIMUM (0)
		0x26, 0x01, 0x00, // LOGICAL_MAXIMUM (1)
		0x95, 0x09, // REPORT_COUNT (9)
		0x75, 0x01, // REPORT_SIZE (1)
		0x81, 0x02, // INPUT (Data,Var,Abs)
		0x95, 0x01, // REPORT_COUNT (1)
		0x75, 0x07, // REPORT_SIZE (7)
		/* USER CODE END 0 */
		0xC0 /* END_COLLECTION */

};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  TEMPLATE_CUSTOM_HID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_Init(void)
{
  return (0);
}

/**
  * @brief  TEMPLATE_CUSTOM_HID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_DeInit(void)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}


/**
  * @brief  TEMPLATE_CUSTOM_HID_Control
  *         Manage the CUSTOM HID class events
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_OutEvent(uint8_t state)
{
	//To copy the reception buffer into the report_buffer variable
	memcpy(report_buffer, state, 64);

	flag_rx = 1;

	return (USBD_OK);
}

#ifdef USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED
/**
  * @brief  TEMPLATE_CUSTOM_HID_CtrlReqComplete
  *         Manage the CUSTOM HID control request complete
  * @param  request: control request
  * @param  wLength: request wLength
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_CtrlReqComplete(uint8_t request, uint16_t wLength)
{
  UNUSED(wLength);

  switch (request)
  {
    case CUSTOM_HID_REQ_SET_REPORT:

      break;

    case CUSTOM_HID_REQ_GET_REPORT:

      break;

    default:
      break;
  }

  return (0);
}
#endif /* USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED */


#ifdef USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED
/**
  * @brief  TEMPLATE_CUSTOM_HID_GetReport
  *         Manage the CUSTOM HID control Get Report request
  * @param  event_idx: event index
  * @param  state: event state
  * @retval return pointer to HID report
  */
static uint8_t *TEMPLATE_CUSTOM_HID_GetReport(uint16_t *ReportLength)
{
  UNUSED(ReportLength);
  uint8_t *pbuff;

  return (pbuff);
}
#endif /* USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED */
