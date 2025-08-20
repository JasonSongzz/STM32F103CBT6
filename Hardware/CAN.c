/******************************************************************************
 * @file    CAN.c
 * @brief   CAN hardware interface implementation
 * @details This file implements the CAN hardware interface for communication
 *          with external devices using Controller Area Network protocol
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "can.h"
#include "CanCmd.h"

/* Global variable to store current baud rate */
static CAN_BaudRate currentBaudRate = CAN_BAUDRATE_500K;

/******************************************************************************
 * @brief   Initialize CAN peripheral
 * @details Configure and initialize CAN controller with specified mode and 
 *          baud rate
 * @param   mode CAN operation mode
 * @param   baud CAN baud rate
 * @return  bool Initialization status
 * @retval  true  Initialization successful
 * @retval  false Initialization failed
 *****************************************************************************/
bool CANInit(CAN_Mode mode, CAN_BaudRate baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Save current baud rate */
    currentBaudRate = baud;

    CAN_InitStructure.CAN_NART = ENABLE; 
    CAN_InitStructure.CAN_TXFP = ENABLE; 
    CAN_InitStructure.CAN_RFLM = DISABLE;   
    CAN_InitStructure.CAN_TTCM = DISABLE;   
    CAN_InitStructure.CAN_ABOM = ENABLE;             

    switch(mode)
    {
        case CAN_MODE_NORMAL:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
            break;
        case CAN_MODE_LOOPBACK:
            CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
            break;
        case CAN_MODE_SILENT:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
            break;
        case CAN_MODE_SILENT_LOOPBACK:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;
            break;
        default:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    }
    
    /* Configure using current baud rate */
    switch(currentBaudRate)
    {
        case CAN_BAUDRATE_5K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 400;
            break;
        case CAN_BAUDRATE_10K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 200;
            break;
        case CAN_BAUDRATE_20K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 100;
            break;
        case CAN_BAUDRATE_50K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 40;
            break;
        case CAN_BAUDRATE_100K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 20;
            break;
        case CAN_BAUDRATE_125K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 16;
            break;
        case CAN_BAUDRATE_250K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 8;
            break;
        case CAN_BAUDRATE_500K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 4;
            break;
        case CAN_BAUDRATE_600K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq;  /* Time segment 1 = 14tq */
            CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;   /* Time segment 2 = 5tq  */
            CAN_InitStructure.CAN_Prescaler = 3;        /* Prescaler             */
            break;
        case CAN_BAUDRATE_750K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;  /* Time segment 1 = 15tq */
            CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;   /* Time segment 2 = 4tq  */
            CAN_InitStructure.CAN_Prescaler = 3;        /* Prescaler             */
            break;
        case CAN_BAUDRATE_1000K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 2;
            break;
        default:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 4;
    }

    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    
    return true; 
}

/******************************************************************************
 * @brief   Update CAN baud rate dynamically
 * @details Change CAN baud rate without reinitializing peripheral
 * @param   newBaud New CAN baud rate
 * @return  None
 *****************************************************************************/
void CAN_Update_BaudRate(CAN_BaudRate newBaud)
{
    currentBaudRate = newBaud;    /* Save new baud rate */
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);    /* Disable CAN interrupt */
    
    CAN1->MCR |= CAN_MCR_INRQ;    /* Enter initialization mode */
    while ((CAN1->MSR & CAN_MSR_INAK) == 0); /* Wait for initialization mode */
    
    CAN_InitTypeDef CAN_InitStructure;    /* Reconfigure baud rate */
    CAN_StructInit(&CAN_InitStructure);   /* Initialize structure */
    
    /* Preserve existing configuration */
    CAN_InitStructure.CAN_NART = DISABLE; 
    CAN_InitStructure.CAN_TXFP = DISABLE; 
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;    
    CAN_InitStructure.CAN_TTCM = DISABLE;   
    CAN_InitStructure.CAN_ABOM = ENABLE; 
    
    /* Set new baud rate */
    switch(currentBaudRate)
    {
        case CAN_BAUDRATE_5K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 400;
            break;
        case CAN_BAUDRATE_10K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 200;
            break;
        case CAN_BAUDRATE_20K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 100;
            break;
        case CAN_BAUDRATE_50K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 40;
            break;
        case CAN_BAUDRATE_100K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 20;
            break;
        case CAN_BAUDRATE_125K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 16;
            break;
        case CAN_BAUDRATE_250K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 8;
            break;
        case CAN_BAUDRATE_500K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 4;
            break;
        case CAN_BAUDRATE_600K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq;  /* Time segment 1 = 14tq */
            CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;   /* Time segment 2 = 5tq  */
            CAN_InitStructure.CAN_Prescaler = 3;        /* Prescaler             */
            break;
        case CAN_BAUDRATE_750K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;  /* Time segment 1 = 15tq */
            CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;   /* Time segment 2 = 4tq  */
            CAN_InitStructure.CAN_Prescaler = 3;        /* Prescaler             */
            break;
        case CAN_BAUDRATE_1000K:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 2;
            break;
        default:
            CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
            CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
            CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
            CAN_InitStructure.CAN_Prescaler = 4;
    }
    
    CAN_Init(CAN1, &CAN_InitStructure);
    
    CAN1->MCR &= ~CAN_MCR_INRQ;    /* Exit initialization mode */
    while (CAN1->MSR & CAN_MSR_INAK); /* Wait for exit from init mode */
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);    /* Re-enable interrupt */
    
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

/******************************************************************************
 * @brief   Convert baud number to baud rate enumeration
 * @details Map numeric baud rate identifier to CAN_BaudRate enumeration value
 * @param   baudNum Numeric baud rate identifier
 * @return  CAN_BaudRate Corresponding baud rate enumeration value
 *****************************************************************************/
CAN_BaudRate baudNumToBaudRate(uint8_t baudNum)
{
    switch(baudNum) 
    {
        case 0: return CAN_BAUDRATE_5K;
        case 1: return CAN_BAUDRATE_10K;
        case 2: return CAN_BAUDRATE_20K;
        case 3: return CAN_BAUDRATE_50K;
        case 4: return CAN_BAUDRATE_100K;
        case 5: return CAN_BAUDRATE_125K;
        case 6: return CAN_BAUDRATE_250K;
        case 7: return CAN_BAUDRATE_500K;
        case 8: return CAN_BAUDRATE_600K; 
        case 9: return CAN_BAUDRATE_750K;
        case 10: return CAN_BAUDRATE_1000K;
        default: return CAN_BAUDRATE_500K;/* Default value */
    }
}

/******************************************************************************
 * @brief   Get current CAN baud rate
 * @details Return the currently configured CAN baud rate
 * @param   None
 * @return  CAN_BaudRate Current baud rate
 *****************************************************************************/
CAN_BaudRate GetCurrentBaudRate(void)
{
    CAN_BaudRate ret = currentBaudRate;
    return ret;
}

/******************************************************************************
 * @brief   CAN TX interrupt handler
 * @details Handle CAN transmit interrupt
 * @param   None
 * @return  None
 *****************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
        can_tx_handle();
    }
}

/******************************************************************************
 * @brief   CAN RX interrupt handler
 * @details Handle CAN receive interrupt
 * @param   None
 * @return  None
 *****************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        can_rx_handle();
    }
}
