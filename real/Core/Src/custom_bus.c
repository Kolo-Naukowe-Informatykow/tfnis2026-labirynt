/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : custom_bus.c
  * @brief          : source file for the BSP BUS IO driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "custom_bus.h"

__weak HAL_StatusTypeDef MX_SPI3_Init(SPI_HandleTypeDef* hspi);
__weak HAL_StatusTypeDef MX_I2C2_Init(I2C_HandleTypeDef* hi2c);

/** @addtogroup BSP
  * @{
  */

/** @addtogroup CUSTOM
  * @{
  */

/** @defgroup CUSTOM_BUS CUSTOM BUS
  * @{
  */

/** @defgroup CUSTOM_BUS_Exported_Variables BUS Exported Variables
  * @{
  */

SPI_HandleTypeDef hspi3;
I2C_HandleTypeDef hi2c2;
/**
  * @}
  */

/** @defgroup CUSTOM_BUS_Private_Variables BUS Private Variables
  * @{
  */

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
static uint32_t IsSPI3MspCbValid = 0;
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
static uint32_t SPI3InitCounter = 0;
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
static uint32_t IsI2C2MspCbValid = 0;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
static uint32_t I2C2InitCounter = 0;

/**
  * @}
  */

/** @defgroup CUSTOM_BUS_Private_FunctionPrototypes  BUS Private Function
  * @{
  */

static void SPI3_MspInit(SPI_HandleTypeDef* hSPI);
static void SPI3_MspDeInit(SPI_HandleTypeDef* hSPI);
#if (USE_CUBEMX_BSP_V2 == 1)
static uint32_t SPI_GetPrescaler( uint32_t clk_src_hz, uint32_t baudrate_mbps );
#endif
static void I2C2_MspInit(I2C_HandleTypeDef* hI2c);
static void I2C2_MspDeInit(I2C_HandleTypeDef* hI2c);
#if (USE_CUBEMX_BSP_V2 == 1)
static uint32_t I2C_GetTiming(uint32_t clock_src_hz, uint32_t i2cfreq_hz);
static void Compute_PRESC_SCLDEL_SDADEL(uint32_t clock_src_freq, uint32_t I2C_Speed);
static uint32_t Compute_SCLL_SCLH (uint32_t clock_src_freq, uint32_t I2C_speed);
#endif

/**
  * @}
  */

/** @defgroup CUSTOM_LOW_LEVEL_Private_Functions CUSTOM LOW LEVEL Private Functions
  * @{
  */

/** @defgroup CUSTOM_BUS_Exported_Functions CUSTOM_BUS Exported Functions
  * @{
  */

/* BUS IO driver over SPI Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER SPI
*******************************************************************************/
/**
  * @brief  Initializes SPI HAL.
  * @retval BSP status
  */
int32_t BSP_SPI3_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  hspi3.Instance  = SPI3;

  if(SPI3InitCounter++ == 0)
  {
    if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_RESET)
    {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0U)
        /* Init the SPI Msp */
        SPI3_MspInit(&hspi3);
#else
        if(IsSPI3MspCbValid == 0U)
        {
            if(BSP_SPI3_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
            {
                return BSP_ERROR_MSP_FAILURE;
            }
        }
#endif
        if(ret == BSP_ERROR_NONE)
        {
            /* Init the SPI */
            if (MX_SPI3_Init(&hspi3) != HAL_OK)
            {
                ret = BSP_ERROR_BUS_FAILURE;
            }
        }
    }
  }

  return ret;
}

/**
  * @brief  DeInitializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BSP_SPI3_DeInit(void)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  if (SPI3InitCounter > 0)
  {
    if (--SPI3InitCounter == 0)
    {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0U)
      SPI3_MspDeInit(&hspi3);
#endif
      /* DeInit the SPI*/
      if (HAL_SPI_DeInit(&hspi3) == HAL_OK)
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }
  return ret;
}

/**
  * @brief  Write Data through SPI BUS.
  * @param  pData: Pointer to data buffer to send
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI3_Send(uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if(HAL_SPI_Transmit(&hspi3, pData, Length, BUS_SPI3_POLL_TIMEOUT) != HAL_OK)
  {
      ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData: Pointer to data buffer to receive
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t  BSP_SPI3_Recv(uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if(HAL_SPI_Receive(&hspi3, pData, Length, BUS_SPI3_POLL_TIMEOUT) != HAL_OK)
  {
      ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  return ret;
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Pointer to data buffer to send/receive
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI3_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if(HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, Length, BUS_SPI3_POLL_TIMEOUT) != HAL_OK)
  {
      ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  return ret;
}

/* BUS IO driver over I2C Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief  Initialize I2C HAL
  * @retval BSP status
  */
int32_t BSP_I2C2_Init(void)
{

  int32_t ret = BSP_ERROR_NONE;

  hi2c2.Instance  = I2C2;

  if(I2C2InitCounter++ == 0)
  {
    if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_RESET)
    {
    #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0U)
      /* Init the I2C Msp */
      I2C2_MspInit(&hi2c2);
    #else
      if(IsI2C2MspCbValid == 0U)
      {
        if(BSP_I2C2_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
        {
          return BSP_ERROR_MSP_FAILURE;
        }
      }
    #endif
      if(ret == BSP_ERROR_NONE)
      {
        /* Init the I2C */
        if(MX_I2C2_Init(&hi2c2) != HAL_OK)
        {
          ret = BSP_ERROR_BUS_FAILURE;
        }
        else if(HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        {
          ret = BSP_ERROR_BUS_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
    }
  }
  return ret;
}

/**
  * @brief  DeInitialize I2C HAL.
  * @retval BSP status
  */
int32_t BSP_I2C2_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  if (I2C2InitCounter > 0)
  {
    if (--I2C2InitCounter == 0)
    {
  #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0U)
      /* DeInit the I2C */
      I2C2_MspDeInit(&hi2c2);
  #endif
      /* DeInit the I2C */
      if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }
    }
  }
  return ret;
}

/**
  * @brief  Check whether the I2C bus is ready.
  * @param DevAddr : I2C device address
  * @param Trials : Check trials number
  * @retval BSP status
  */
int32_t BSP_I2C2_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_IsDeviceReady(&hi2c2, DevAddr, Trials, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    ret = BSP_ERROR_BUSY;
  }

  return ret;
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP status
  */

int32_t BSP_I2C2_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Write(&hi2c2, DevAddr,Reg, I2C_MEMADD_SIZE_8BIT,pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BSP_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Read(&hi2c2, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**

  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write

  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP statu
  */
int32_t BSP_I2C2_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Write(&hi2c2, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BSP_I2C2_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Read(&hi2c2, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
    {
      ret =  BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C2_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Master_Transmit(&hi2c2, DevAddr, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C2_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Master_Receive(&hi2c2, DevAddr, pData, Length, BUS_I2C2_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
/**
  * @brief Register Default BSP SPI3 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_SPI3_RegisterDefaultMspCallbacks (void)
{

  __HAL_SPI_RESET_HANDLE_STATE(&hspi3);

  /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPINIT_CB_ID, SPI3_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPDEINIT_CB_ID, SPI3_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsSPI3MspCbValid = 1;

  return BSP_ERROR_NONE;
}

/**
  * @brief BSP SPI3 Bus Msp Callback registering
  * @param Callbacks     pointer to SPI3 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_SPI3_RegisterMspCallbacks (BSP_SPI_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_SPI_RESET_HANDLE_STATE(&hspi3);

   /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPINIT_CB_ID, Callbacks->pMspInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPDEINIT_CB_ID, Callbacks->pMspDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  IsSPI3MspCbValid = 1;

  return BSP_ERROR_NONE;
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/**
  * @brief Register Default BSP I2C2 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_I2C2_RegisterDefaultMspCallbacks (void)
{

  __HAL_I2C_RESET_HANDLE_STATE(&hi2c2);

  /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MSPINIT_CB_ID, I2C2_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MSPDEINIT_CB_ID, I2C2_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsI2C2MspCbValid = 1;

  return BSP_ERROR_NONE;
}

/**
  * @brief BSP I2C2 Bus Msp Callback registering
  * @param Callbacks     pointer to I2C2 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_I2C2_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_I2C_RESET_HANDLE_STATE(&hi2c2);

   /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MSPINIT_CB_ID, Callbacks->pMspInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MSPDEINIT_CB_ID, Callbacks->pMspDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  IsI2C2MspCbValid = 1;

  return BSP_ERROR_NONE;
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BSP_GetTick(void) {
  return HAL_GetTick();
}

/* SPI3 init function */

__weak HAL_StatusTypeDef MX_SPI3_Init(SPI_HandleTypeDef* hspi)
{
  HAL_StatusTypeDef ret = HAL_OK;

  hspi->Instance = SPI3;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 0x7;
  hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi->Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi->Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi->Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi->Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi->Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi->Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi->Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi->Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

static void SPI3_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = BUS_SPI3_SCK_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BUS_SPI3_SCK_GPIO_AF;
    HAL_GPIO_Init(BUS_SPI3_SCK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_SPI3_MISO_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BUS_SPI3_MISO_GPIO_AF;
    HAL_GPIO_Init(BUS_SPI3_MISO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_SPI3_MOSI_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BUS_SPI3_MOSI_GPIO_AF;
    HAL_GPIO_Init(BUS_SPI3_MOSI_GPIO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
}

static void SPI3_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(BUS_SPI3_SCK_GPIO_PORT, BUS_SPI3_SCK_GPIO_PIN);

    HAL_GPIO_DeInit(BUS_SPI3_MISO_GPIO_PORT, BUS_SPI3_MISO_GPIO_PIN);

    HAL_GPIO_DeInit(BUS_SPI3_MOSI_GPIO_PORT, BUS_SPI3_MOSI_GPIO_PIN);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
}

/* I2C2 init function */

__weak HAL_StatusTypeDef MX_I2C2_Init(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef ret = HAL_OK;

  hi2c->Instance = I2C2;
  hi2c->Init.Timing = 0x10C043E5;
  hi2c->Init.OwnAddress1 = 0;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c->Init.OwnAddress2 = 0;
  hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

static void I2C2_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB12     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = BUS_I2C2_SCL_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BUS_I2C2_SCL_GPIO_AF;
    HAL_GPIO_Init(BUS_I2C2_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_I2C2_SDA_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BUS_I2C2_SDA_GPIO_AF;
    HAL_GPIO_Init(BUS_I2C2_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
}

static void I2C2_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB12     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(BUS_I2C2_SCL_GPIO_PORT, BUS_I2C2_SCL_GPIO_PIN);

    HAL_GPIO_DeInit(BUS_I2C2_SDA_GPIO_PORT, BUS_I2C2_SDA_GPIO_PIN);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
}

#if (USE_CUBEMX_BSP_V2 == 1)
/**
  * @brief  Convert the SPI baudrate into prescaler.
  * @param  clock_src_hz : SPI source clock in HZ.
  * @param  baudrate_mbps : SPI baud rate in mbps.
  * @retval Prescaler divisor
  */
static uint32_t SPI_GetPrescaler( uint32_t clock_src_hz, uint32_t baudrate_mbps )
{
  uint32_t divisor = 0;
  uint32_t spi_clk = clock_src_hz;
  uint32_t presc = 0;

  static const uint32_t baudrate[]=
  {
    SPI_BAUDRATEPRESCALER_2,
    SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,
    SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,
    SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128,
    SPI_BAUDRATEPRESCALER_256,
  };

  while( spi_clk > baudrate_mbps)
  {
    presc = baudrate[divisor];
    if (++divisor > 7)
      break;

    spi_clk= ( spi_clk >> 1);
  }

  return presc;
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

