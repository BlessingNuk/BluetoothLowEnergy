/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/p2p_routeur_app.h
 * Description        : Header for P2P router application module
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __P2P_ROUTEUR_APP_H
#define __P2P_ROUTEUR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint8_t     Device1_Status;
/* USER CODE BEGIN P2PR_APP_Device_Status_t_Multidevice */

/* USER CODE END P2PR_APP_Device_Status_t_Multidevice */
 }P2PR_APP_Device_Status_t;
 /* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void P2PR_APP_Init( void );
void P2PR_APP_End_Device_Mgt_Connection_Update( P2PR_APP_Device_Status_t *p_device_status );
//void Evt_Notification( P2P_ConnHandle_Not_evt_t *pNotification );
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*__P2P_ROUTEUR_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
