/******************************************************************************
 * @file    CanCmd.c
 * @brief   CAN command processing implementation
 * @details This file implements the CAN communication protocol handling 
 *          including command processing, data packaging, and buffer management
 * @author  
 * @date    
 * @version 1.0
 *****************************************************************************/
#include "CanCmd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "CAN.h"
#include "Date.h"
#include "bsp_adxl345_driver.h"
#include "bsp_pt100_driver.h"
#include "bsp_water_driver.h"
#include "bsp_tsh54_driver.h"
#include "bsp_a22_driver.h"

Param_t Param = {
    .BaudNum = 7,
    .SensorId = 1,
};

/* Global instances */
static CANRxBuf_t rxBufCtrl;
static CANTxBuf_t txBufCtrl;
static uint8_t RecvPackBuf[RX_BUF_LEN] = {0};

/******************************************************************************
 * @brief   Initialize CAN buffers
 * @details Initialize transmit and receive buffers
 * @param   None
 * @return  None
 *****************************************************************************/
void can_buffer_init(void)
{
    memset(&txBufCtrl, 0, sizeof(txBufCtrl));
    memset(&rxBufCtrl, 0, sizeof(rxBufCtrl));
}

/******************************************************************************
 * @brief   Calculate available space in transmit buffer
 * @details Calculate the amount of free space in the transmit buffer
 * @param   buf Pointer to transmit buffer control structure
 * @return  uint16_t Available space in bytes
 *****************************************************************************/
static uint16_t cal_available_space(const CANTxBuf_t* buf)
{
    if(buf->DataIn >= buf->DataOut) 
    {
        return TX_BUF_LEN - 1 - (buf->DataIn - buf->DataOut);
    }else{
        return buf->DataOut - buf->DataIn - 1;
    }
}

/******************************************************************************
 * @brief   Write data to transmit buffer
 * @details Write specified amount of data to the transmit buffer
 * @param   buf Pointer to transmit buffer control structure
 * @param   data Pointer to data to write
 * @param   len  Length of data to write
 * @return  None
 *****************************************************************************/
static void write_to_buffer(CANTxBuf_t* buf, const uint8_t* data, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) 
    {
        buf->DataBuf[buf->DataIn] = data[i];
        buf->DataIn = (buf->DataIn + 1) % TX_BUF_LEN;
    }
}

/******************************************************************************
 * @brief   Write protocol header to transmit buffer
 * @details Convert protocol header structure to byte stream and write to buffer
 * @param   buf    Pointer to transmit buffer control structure
 * @param   header Pointer to protocol header structure
 * @return  None
 *****************************************************************************/
static void write_protocol_header(CANTxBuf_t* buf, const ProtocolHead_t* header)
{
    /* Convert structure to byte stream */
    uint8_t headerBytes[PROTOCOL_HEAD_SIZE];
    memcpy(headerBytes, header, PROTOCOL_HEAD_SIZE);
    
    write_to_buffer(buf, headerBytes, PROTOCOL_HEAD_SIZE);
}

/******************************************************************************
 * @brief   Calculate and write checksum
 * @details Calculate checksum of specified data and write to buffer
 * @param   buf         Pointer to transmit buffer control structure
 * @param   startIndex  Start index for checksum calculation
 * @param   dataLength  Length of data to calculate checksum for
 * @return  None
 *****************************************************************************/
static void cal_and_write_checksum(CANTxBuf_t* buf, uint16_t startIndex, 
                                      uint16_t dataLength)
{
    uint8_t checknum = 0;
    uint16_t currentIndex = startIndex;
    
    for(uint16_t i = 0; i < dataLength; i++) 
    {
        checknum += buf->DataBuf[currentIndex];
        currentIndex = (currentIndex + 1) % TX_BUF_LEN;
    }
    /* Write checksum */
    write_to_buffer(buf, &checknum, 1);
}

/******************************************************************************
 * @brief   Send command
 * @details Send command with specified parameters via CAN
 * @param   p_SendInfo Pointer to send information structure
 * @return  uint8_t Send status
 * @retval  1 Send successful
 * @retval  0 Send failed
 *****************************************************************************/
static uint8_t send_cmd(SendInfo_t *p_SendInfo)
{
    uint8_t retryCount = 0;
    uint16_t requiredSpace = p_SendInfo->SendLen + PROTOCOL_HEAD_SIZE + 2; 
    uint16_t startIndex = txBufCtrl.DataIn;
    
    do{
        uint16_t availableSpace = cal_available_space(&txBufCtrl);
        if(availableSpace >= requiredSpace) break;
        
        /* Try to send multiple frames on each retry */
        for(int i=0; i<5; i++) 
        {
            can_tx_handle();
            availableSpace = cal_available_space(&txBufCtrl);
            if(availableSpace >= requiredSpace) break;
        }
        
        if(++retryCount >= 5)
        {
            return 0;
        }
    }while(1);
    /* Create local protocol header structure */
    ProtocolHead_t header = {
        .Head = PROTOCOL_HEAD,
        .DataLen = p_SendInfo->SendLen,
        .ProtocolVer = PROTOCOL_VER,
        .ProductType = PRODUCT_TYPE,
        .SrcId = p_SendInfo->SendId,
        .DstId = p_SendInfo->DstId,
        .Cmd = p_SendInfo->Cmd
    };
    /* Write protocol header */
    write_protocol_header(&txBufCtrl, &header);
    
    /* Write data */
    write_to_buffer(&txBufCtrl, p_SendInfo->pbuf, p_SendInfo->SendLen);
    
    /* Calculate and write checksum */
    cal_and_write_checksum(&txBufCtrl, startIndex, 
                              PROTOCOL_HEAD_SIZE + p_SendInfo->SendLen);
    
    /* Write protocol tail */
    uint8_t tail = PROTOCOL_TAIL;
    write_to_buffer(&txBufCtrl, &tail, 1);
    
    /* Save send ID */
    txBufCtrl.SendId = p_SendInfo->SendId;
    
    /* Start transmission */
    can_tx_handle();
    return 1;
}

/******************************************************************************
 * @brief   Handle host heartbeat command
 * @details Process heartbeat command from host and send response
 * @param   pData   Pointer to received data
 * @param   DataLen Length of received data
 * @return  None
 *****************************************************************************/
static void host_heart_bate_cmd(uint8_t *pData, uint16_t DataLen)
{
    if(pData == NULL || DataLen < sizeof(p_clock_t)) 
    {
        return;
    }
    p_clock_t clockData;
    memcpy(&clockData, pData, sizeof(p_clock_t));
    
    SetDateToTick(&clockData);
    
    /* Prepare send information */
    SendInfo_t SendInfo = {
        .DstId = PROTOCOL_DST_ID,
        .SendId = Param.SensorId,
        .SendLen = 1,
        .pbuf = &(uint8_t){0},
        .Cmd = CMD_HEART_BEAT
    };
    send_cmd(&SendInfo);
}

/******************************************************************************
 * @brief   Create packed sensor data
 * @details Pack all sensor data into a single buffer for transmission
 * @param   dataSize Pointer to store total data size
 * @return  uint8_t* Pointer to packed data buffer
 * @retval  Non-NULL Pointer to packed data
 * @retval  NULL     Memory allocation failed
 *****************************************************************************/
static uint8_t* create_packed_data(uint16_t* dataSize)
{	

	pt100_driver_t pt100_driver = pt100_get_driver();
    water_driver_t water_driver = water_get_driver();
	tsh54_driver_t tsh54_driver = tsh54_get_driver();
    adxl345_driver_t adxl345_driver = adxl345_get_driver();
    a22_driver_t a22_driver = a22_get_driver();
    
    /* Calculate total size */
    uint16_t totalSize = sizeof(uint8_t); /* ChannelCnt */
    
    /* PT100 TEMP (1 value) */
    totalSize += sizeof(uint8_t) * 4 + sizeof(float) * 1; /* 4 uint8_t + 1 float */
    
    /* TSH54 RPM (1 value) */
    totalSize += sizeof(uint8_t) * 4 + sizeof(float) * 1;
    
    /* WATER status (1 value) */
    totalSize += sizeof(uint8_t) * 4 + sizeof(float) * 1;
    
    /* A22 DISTANCE (1 value) */
    totalSize += sizeof(uint8_t) * 4 + sizeof(float) * 1;
    
    /* ADXL345 (3 values) */
    totalSize += sizeof(uint8_t) * 4 + sizeof(float) * 3; /* 4 uint8_t + 3 floats */
    
    /* Allocate memory */
    uint8_t* packedData = malloc(totalSize);
    if (!packedData) return NULL;
    
    uint8_t* ptr = packedData;
    
    /* Write ChannelCnt */
    *ptr = 5;  /* 5 channels */
    ptr += sizeof(uint8_t);
    
    /* PT100 TEMP */
    *ptr++ = 1; *ptr++ = 1; *ptr++ = DATA_RATIO_1; *ptr++ = FLOAT_VALUE;
    *((float*)ptr) = pt100_driver.data.temperature;
    ptr += sizeof(float);
    
    /* TSH54 RPM */
    *ptr++ = 1; *ptr++ = 2; *ptr++ = DATA_RATIO_1; *ptr++ = FLOAT_VALUE;
    *((float*)ptr) = tsh54_driver.data.pulse_count;
    ptr += sizeof(float);
    
    /* WATER status */
    *ptr++ = 1; *ptr++ = 3; *ptr++ = DATA_RATIO_1; *ptr++ = FLOAT_VALUE;
    *((float*)ptr) = water_driver.data.resistance;
    ptr += sizeof(float);
    
    /* A22 DISTANCE */
    *ptr++ = 1; *ptr++ = 4; *ptr++ = DATA_RATIO_1; *ptr++ = FLOAT_VALUE;
    *((float*)ptr) = a22_driver.data.distance_mm;
    ptr += sizeof(float);
    
    /* ADXL345 */
    *ptr++ = 3; *ptr++ = 5; *ptr++ = DATA_RATIO_1; *ptr++ = FLOAT_VALUE;
    *((float*)ptr) = adxl345_driver.data.x; ptr += sizeof(float);
    *((float*)ptr) = adxl345_driver.data.y; ptr += sizeof(float);
    *((float*)ptr) = adxl345_driver.data.z; ptr += sizeof(float);
    
    *dataSize = totalSize;
    return packedData;
}

/******************************************************************************
 * @brief   Handle host read data command
 * @details Process read sensor data command from host
 * @param   pData   Pointer to received data
 * @param   DataLen Length of received data
 * @return  None
 *****************************************************************************/
static void host_read_data_cmd(uint8_t *pData, uint16_t DataLen)
{
    uint16_t packedSize;
    uint8_t* packedData = create_packed_data(&packedSize);
    
    if (packedData) {
        SendInfo_t SendInfo = {
            .DstId = PROTOCOL_DST_ID,
            .SendId = Param.SensorId,
            .SendLen = packedSize,
            .pbuf = packedData,
            .Cmd = CMD_READ_SENSOR_DATA
        }; 
        send_cmd(&SendInfo);
        free(packedData);
    }
}

/******************************************************************************
 * @brief   Handle host read ID command
 * @details Process read sensor ID command from host
 * @param   pData   Pointer to received data
 * @param   DataLen Length of received data
 * @return  None
 *****************************************************************************/
static void host_read_id_cmd(uint8_t *pData, uint16_t DataLen)
{
    /* Create send information structure */
    SendInfo_t SendInfo = {
        .Cmd = CMD_READ_SENSOR_ID,
        .DstId = PROTOCOL_DST_ID,
        .SendId = Param.SensorId,
        .SendLen = sizeof(Param.SensorId),
        .pbuf = (uint8_t *)&Param.SensorId
    };
    send_cmd(&SendInfo);
}

/******************************************************************************
 * @brief   Handle host set ID command
 * @details Process set sensor ID command from host
 * @param   pData   Pointer to received data
 * @param   DataLen Length of received data
 * @return  None
 *****************************************************************************/
static void host_set_id_cmd(uint8_t *pData, uint16_t DataLen)
{
    if(pData == NULL || DataLen < sizeof(uint16_t)) 
    {
        return;
    }
    
    uint16_t newId;
    uint16_t oldId = Param.SensorId;
    memcpy(&newId, pData, sizeof(uint16_t));
    
    /* Update device ID */
    Param.SensorId = newId;
    
    /* Prepare acknowledge response */
    SetId_t AckData = { .CanId = newId };
    
    /* Create send information structure */
    SendInfo_t SendInfo = {
        .Cmd = CMD_SET_SENSOR_ID,
        .DstId = PROTOCOL_DST_ID,
        .SendId = oldId, /* Source ID is old ID to prevent host from losing device */
        .SendLen = sizeof(AckData),
        .pbuf = (uint8_t *)&AckData
    };
    send_cmd(&SendInfo);
}

/******************************************************************************
 * @brief   Handle host read baud command
 * @details Process read sensor baud rate command from host
 * @param   pData   Pointer to received data
 * @param   DataLen Length of received data
 * @return  None
 *****************************************************************************/
static void host_read_baud_cmd(uint8_t *pData, uint16_t DataLen)
{
    /* Create send information structure */
    SendInfo_t SendInfo = {
        .Cmd = CMD_READ_SENSOR_BAUD,
        .DstId = PROTOCOL_DST_ID,
        .SendId = Param.SensorId,
        .SendLen = sizeof(Param.BaudNum),
        .pbuf = &Param.BaudNum
    };
    send_cmd(&SendInfo);
}

/******************************************************************************
 * @brief   Handle host set baud command
 * @details Process set sensor baud rate command from host
 * @param   pData   Pointer to received data
 * @param   DataLen Length of received data
 * @return  None
 *****************************************************************************/
static void host_set_baud_cmd(uint8_t *pData, uint16_t DataLen)
{
    if(pData == NULL || DataLen < sizeof(uint8_t)) 
    {
        return;
    }
    /* Get new baud rate number */
    uint8_t newBaudNum = *pData;
    
    /* Update global parameter */
    Param.BaudNum = newBaudNum;
    
    /* Convert to baud rate enumeration value */
    CAN_BaudRate newBaud = baudNumToBaudRate(newBaudNum);
    
    /* Update CAN baud rate */
    CAN_Update_BaudRate(newBaud);
    
    /* Send success response */    
    uint8_t successCode = 1; 
    if(GetCurrentBaudRate() == newBaud)
    {
        successCode = 0;
    }
    /* Create send information structure */
    SendInfo_t SendInfo = {
        .Cmd = CMD_SET_SENSOR_BAUD,
        .DstId = PROTOCOL_DST_ID,
        .SendId = Param.SensorId,
        .SendLen = sizeof(successCode),
        .pbuf = &successCode
    };
    send_cmd(&SendInfo);
}

ProcCmd_t ProcCmdList[] = 
{  
    {CMD_HEART_BEAT,       host_heart_bate_cmd},      
    {CMD_READ_SENSOR_DATA, host_read_data_cmd},          
    {CMD_READ_SENSOR_ID,   host_read_id_cmd},            
    {CMD_SET_SENSOR_ID,    host_set_id_cmd},             
    {CMD_READ_SENSOR_BAUD, host_read_baud_cmd},        
    {CMD_SET_SENSOR_BAUD,  host_set_baud_cmd},
    {0xFF, NULL},
};

/******************************************************************************
 * @brief   Check received data
 * @details Validate and process received data from buffer
 * @param   pInfo Pointer to receive information structure
 * @return  uint8_t Check status
 * @retval  1 Data valid
 * @retval  0 Data invalid
 *****************************************************************************/
static uint8_t check_recv_data(RecvInfo_t *pInfo)
{
    uint8_t i = 0, CheckNum1 = 0, CheckNum2 = 0;
    uint16_t DataLen = 0;
    
    /* Check if buffer is empty */
    if(rxBufCtrl.RecvIn == rxBufCtrl.RecvOut)
    {
        return 0;
    }
    
    /* Find protocol header */
    while(rxBufCtrl.RecvIn != rxBufCtrl.RecvOut)
    {
        if (rxBufCtrl.RecvBuf[rxBufCtrl.RecvOut] == PROTOCOL_HEAD)
        {
            break;
        }
        rxBufCtrl.RecvOut = (rxBufCtrl.RecvOut + 1) % RX_BUF_LEN;
    }
    
    if ((((rxBufCtrl.RecvIn + RX_BUF_LEN) - rxBufCtrl.RecvOut) % RX_BUF_LEN) < 
        PROTOCOL_HEAD_SIZE)
    {
        return 0;
    }
    
    DataLen = ((uint16_t)rxBufCtrl.RecvBuf[(rxBufCtrl.RecvOut + 2) % RX_BUF_LEN] 
               << 8) | rxBufCtrl.RecvBuf[(rxBufCtrl.RecvOut + 1) % RX_BUF_LEN];
    
    if ((((rxBufCtrl.RecvIn + RX_BUF_LEN) - rxBufCtrl.RecvOut) % RX_BUF_LEN) < 
        (DataLen + PROTOCOL_HEAD_SIZE + 2))
    {
        return 0;
    }
    
    /* Calculate checksum */
    for(i = 0; i < (DataLen + PROTOCOL_HEAD_SIZE); i++) 
    {
        uint16_t index = (rxBufCtrl.RecvOut + i) % RX_BUF_LEN;
        CheckNum1 += rxBufCtrl.RecvBuf[index];
    }
    
    /* Get checksum from frame */
    uint16_t checksumIndex = (rxBufCtrl.RecvOut + DataLen + PROTOCOL_HEAD_SIZE) % 
                             RX_BUF_LEN;
    CheckNum2 = rxBufCtrl.RecvBuf[checksumIndex];
    if(CheckNum1 != CheckNum2)
    {
        return 0;
    }
    
    /* Check protocol tail */
    uint16_t tailIndex = (rxBufCtrl.RecvOut + DataLen + PROTOCOL_HEAD_SIZE + 1) % 
                         RX_BUF_LEN;
    if(PROTOCOL_TAIL != rxBufCtrl.RecvBuf[tailIndex]) 
    {
        rxBufCtrl.RecvOut = (rxBufCtrl.RecvOut + 1) % RX_BUF_LEN; /* Skip err. frame */
        return 0;
    }
    
    /* Copy complete frame to temporary buffer */
    for(i = 0; i < DataLen+PROTOCOL_HEAD_SIZE+2; i++) 
    {
        uint16_t index = (rxBufCtrl.RecvOut + i) % RX_BUF_LEN;
        RecvPackBuf[i] = rxBufCtrl.RecvBuf[index];
    }
    
    pInfo->PackLen = DataLen+PROTOCOL_HEAD_SIZE+2;
    pInfo->DataBuf = RecvPackBuf;
    return 1;
}

/******************************************************************************
 * @brief   Parse received packet
 * @details Parse and process received packet based on command
 * @param   pInfo Pointer to receive information structure
 * @return  None
 *****************************************************************************/
static void parse_recv_pack(RecvInfo_t *pInfo)
{
    uint8_t *pData = pInfo->DataBuf;
    ProtocolHead_t *pHead = (ProtocolHead_t *)pData;
    
    /* Process command */
    for(uint8_t i = 0; ProcCmdList[i].Cmd != 0xFF; i++) 
    {
        if (ProcCmdList[i].Cmd == pHead->Cmd) 
        {
            ProcCmdList[i].p(pData + PROTOCOL_HEAD_SIZE, pHead->DataLen);
            break;
        }
    }  
    /* Update receive pointer */
    rxBufCtrl.RecvOut = (rxBufCtrl.RecvOut + pInfo->PackLen) % RX_BUF_LEN;
}

/******************************************************************************
 * @brief   Parse data
 * @details Main data parsing function
 * @param   None
 * @return  None
 *****************************************************************************/
static void data_prase(void)
{
    RecvInfo_t RecvInfo = {0};
    
    if(check_recv_data(&RecvInfo))
    {
        parse_recv_pack(&RecvInfo);
    }
}

/******************************************************************************
 * @brief   Handle CAN transmit operations
 * @details Process transmit buffer and send CAN frames
 * @param   None
 * @return  None
 *****************************************************************************/
void can_tx_handle(void)
{
    CanTxMsg TxMessage = {0};
    
    /* Check if transmit buffer is empty */
    if(txBufCtrl.DataIn == txBufCtrl.DataOut) 
    {
        return;
    }
    TxMessage.StdId = txBufCtrl.SendId;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    /* Calculate data length for this transmission (maximum 8 bytes) */
    uint16_t available = 0;
    if(txBufCtrl.DataIn >= txBufCtrl.DataOut) 
    {
        available = txBufCtrl.DataIn - txBufCtrl.DataOut;
    }else{
        available = TX_BUF_LEN - txBufCtrl.DataOut;
    }
    
    TxMessage.DLC = (available > 8) ? 8 : available;
    
    /* Fill data */
    for(uint8_t i = 0; i < TxMessage.DLC; i++) 
    {
        TxMessage.Data[i] = txBufCtrl.DataBuf[txBufCtrl.DataOut];
        txBufCtrl.DataOut = (txBufCtrl.DataOut + 1) % TX_BUF_LEN;
        
    }
    /* Send CAN frame */
    CAN_Transmit(CAN1, &TxMessage);
}

/******************************************************************************
 * @brief   Handle CAN receive operations
 * @details Process received CAN frames and store in receive buffer
 * @param   None
 * @return  None
 *****************************************************************************/
void can_rx_handle(void)
{ 
    /* Receive data from CAN */
    if(CAN_MessagePending(CAN1, CAN_FIFO0) > 0) 
    {
        CanRxMsg RxMessage = {0};
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        /* Store data in receive buffer */
        for (uint8_t i = 0; i < RxMessage.DLC; i++) 
        {
            /* Check if buffer is full */
            uint16_t nextRecvIn = (rxBufCtrl.RecvIn + 1) % RX_BUF_LEN;
            if (nextRecvIn == rxBufCtrl.RecvOut) 
            {
                /* Buffer full, overwrite oldest data */
                rxBufCtrl.RecvOut = (rxBufCtrl.RecvOut + 1) % RX_BUF_LEN;
            }
            rxBufCtrl.RecvBuf[rxBufCtrl.RecvIn] = RxMessage.Data[i];
            rxBufCtrl.RecvIn = nextRecvIn;
        }
        
        /* Check FIFO overflow flag */
        if(CAN_GetFlagStatus(CAN1, CAN_FLAG_FOV0) == SET) 
        {
            CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
        }
    }
}

/******************************************************************************
 * @brief   Send data via CAN API
 * @details Send data with specified command via CAN bus using API
 * @param   pData Pointer to data buffer
 * @param   Len   Length of data to send
 * @param   Cmd   Command code
 * @return  int8_t Send status
 * @retval  1 Send successful
 * @retval  0 Send failed
 *****************************************************************************/
int8_t api_can_send(uint8_t *pData, uint16_t Len, uint8_t Cmd)
{
    uint8_t Ret = 0;
    SendInfo_t SendInfo = {0};
    SendInfo.Cmd = Cmd;
    SendInfo.DstId = PROTOCOL_DST_ID;
    SendInfo.SendId = Param.SensorId;
    SendInfo.pbuf = pData;
    SendInfo.SendLen = Len;
    
    Ret = send_cmd(&SendInfo);
    return Ret;
}

#if CAN_TEST_ENABLE
uint8_t TestSendLock = 0;
static void CanTest(void)
{
    uint8_t TestSendBuf[] = CAN_TEST_SEND_STRING;
    
    SendInfo_t SendInfo = {0};
    SendInfo.DstId = 0x7FF;
    SendInfo.SendId = 0x01;
    SendInfo.SendLen = strlen(CAN_TEST_SEND_STRING);
    SendInfo.pbuf = TestSendBuf;
    send_cmd(&SendInfo);
}

void TestUnLock(void)
{
    TestSendLock = 1;
}
#endif

/******************************************************************************
 * @brief   Run CAN processing
 * @details Main CAN processing function, handles received data parsing
 * @param   None
 * @return  None
 *****************************************************************************/
void can_task(void)
{
    #if CAN_TEST_ENABLE
    if (TestSendLock == 1)
    {
        TestSendLock = 0;
        CanTest();
    }
    #endif
    data_prase();
}
