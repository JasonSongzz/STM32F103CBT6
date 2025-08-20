/******************************************************************************
 * @file CanCmd.h
 * @brief CAN command processing interface
 * @details This header file defines the CAN communication protocol interface
 *          including command definitions, data structures, and function 
 *          declarations
 *****************************************************************************/

#ifndef __CANCMD_H
#define __CANCMD_H

#include "stm32f10x.h"                  /* Device header                 */
#include "Date.h"
#include "stdbool.h"

#pragma pack(1)

/* Protocol constants */
#define PROTOCOL_HEAD           0x5A    /* Protocol header byte              */
#define PROTOCOL_TAIL           0x97    /* Protocol tail byte                */
#define PROTOCOL_VER            1       /* Protocol version                  */
#define PRODUCT_TYPE            0xEC    /* Product type identifier           */
#define PROTOCOL_SRC_ID         0x0001  /* Default source ID                 */
#define PROTOCOL_DST_ID         0x07FE  /* Default destination ID            */

/* Command definitions */
#define CMD_RESET_SYSTEM        0x17    /* System reset command              */
#define CMD_READ_FACTORY        0x18    /* Read factory info command         */
#define CMD_HEART_BEAT          0x19    /* Heartbeat command                 */
#define CMD_READ_SENSOR_ID      0xA0    /* Read sensor ID command            */
#define CMD_SET_SENSOR_ID       0xA1    /* Set sensor ID command             */
#define CMD_READ_SENSOR_BAUD    0xAA    /* Read sensor baud rate command     */
#define CMD_SET_SENSOR_BAUD     0xA9    /* Set sensor baud rate command      */
#define CMD_READ_SENSOR_DATA    0x21    /* Read sensor data command          */
#define CMD_SET_CHN_DATA        0x22    /* Set channel data command          */
#define CMD_READ_FILE_LIST      0x23    /* Read file list command            */
#define CMD_READ_FILE_DATA      0x24    /* Read file data command            */
#define CMD_DELETE_FILE         0x25    /* Delete file command               */

/* Data type definitions */
#define BIT_1_VALUE             1       /* 1-bit data value                  */
#define BIT_2_VALUE             2       /* 2-bit data value                  */
#define CHAR_VALUE              8       /* Character data value              */
#define SHORT_VALUE             16      /* Short integer data value          */
#define INT_VALUE               32      /* Integer data value                */
#define UCHAR_VALUE             18      /* Unsigned character data value     */
#define USHORT_VALUE            116     /* Unsigned short data value         */
#define UINT_VALUE              132     /* Unsigned integer data value       */
#define FLOAT_VALUE             201     /* Float data value                  */
#define DOUBLE_VALUE            202     /* Double data value                 */

/* File constants */
#define FILE_VER                1       /* File version                      */
#define JHD_DATA                "jhddata" /* Data identifier                 */
#define DIODECUR_CURVE          26      /* Diode curve identifier            */

/* Buffer sizes */
#define TX_BUF_LEN              (4 * 1024) /* Transmit buffer length (4KB)   */
#define RX_BUF_LEN              (1024)  /* Receive buffer length (1KB)       */
#define PROTOCOL_HEAD_SIZE      sizeof(ProtocolHead_t)      /* Protocol head */
#define CURVE_HEAD_SIZE         sizeof(CurveHead_t)         /* Curve header  */
#define CURVE_CHANNEL_HEAD_SIZE sizeof(CurveChannel_t)      /* Curve channel */

/* CAN test configuration */
#define CAN_TEST_ENABLE         0       /* CAN test enable flag              */
#if CAN_TEST_ENABLE
    #define CAN_TEST_SEND_STRING   "123456789012"  /* Test string            */
    static void CanTest(void);
    void TestUnLock(void);
#endif

/* File type enumeration */
typedef enum
{
    Image = 0,                          /* Image file                        */
    Video = 1,                          /* Video file                        */
    Oil,                                /* Oil data                          */
    Vib,                                /* Vibration data                    */
    Drag,                               /* Drag data                         */
    Drag_rpm,                           /* Drag RPM data                     */
    Relays = 7,                         /* Relay data                        */
    Locking,                            /* Locking data                      */
    Normal = 10,                        /* Normal data                       */
}FILE_TYPE_T;

/* Event type enumeration */
typedef enum
{
    EVENT_TYPE_FEELERGAUGE = 0,         /* Feeler gauge event                */
    EVENT_TYPE_CALIBRATION = 1,         /* Calibration event                 */
    EVENT_TYPE_BD = 2,                  /* BD event                          */
    EVENT_TYPE_GC = 3,                  /* GC event                          */
    EVENT_TYPE_HOSTCMD = 4,             /* Host command event                */
    EVENT_TYPE_CYCLE = 5,               /* Cycle event                       */
    EVENT_TYPE_OIL = 6,                 /* Oil event                         */
}EVENT_ENUM_T;

/* Data ratio enumeration */
typedef enum
{
    DATA_RATIO_0_0_0_0_0_1 = -5,        /* 0.00001 ratio                     */
    DATA_RATIO_0_0_0_0_1,               /* 0.0001 ratio                      */
    DATA_RATIO_0_0_0_1,                 /* 0.001 ratio                       */
    DATA_RATIO_0_0_1,                   /* 0.01 ratio                        */
    DATA_RATIO_0_1,                     /* 0.1 ratio                         */
    DATA_RATIO_1,                       /* 1.0 ratio                         */
    DATA_RATIO_1_0,                     /* 10.0 ratio                        */
    DATA_RATIO_1_0_0,                   /* 100.0 ratio                       */
    DATA_RATIO_1_0_0_0,                 /* 1000.0 ratio                      */
    DATA_RATIO_1_0_0_0_0,               /* 10000.0 ratio                     */
}DATA_RATIO_ENUM_T;

/* Command processing structure */
typedef struct
{
    uint8_t Cmd;                        /* Command code                      */
    void (*p)(uint8_t *pData, uint16_t DataLen); /* Command handler function */
}ProcCmd_t;

/* Send information structure */
typedef struct
{
    uint16_t SendId;                    /* Sender ID                         */
    uint16_t DstId;                     /* Destination ID                    */
    uint16_t SendLen;                   /* Data length                       */
    uint8_t Cmd;                        /* Command code                      */
    uint8_t *pbuf;                      /* Data buffer pointer               */
}SendInfo_t;

/* Receive information structure */
typedef struct
{
    uint8_t PackLen;                    /* Packet length                     */
    uint8_t *DataBuf;                   /* Data buffer pointer               */
}RecvInfo_t;

/* Create date structure */
typedef struct
{
    uint8_t day;                        /* Day                               */
    uint8_t month;                      /* Month                             */
    uint8_t year;                       /* Year                              */
}CreateDate_t;

/* Protocol header structure */
typedef struct
{
    uint8_t Head;                       /* Header byte                       */
    uint16_t DataLen;                   /* Data length                       */
    uint8_t ProtocolVer;                /* Protocol version                  */
    uint8_t ProductType;                /* Product type                      */
    uint16_t SrcId;                     /* Source ID                         */
    uint16_t DstId;                     /* Destination ID                    */
    uint8_t Cmd;                        /* Command code                      */
}ProtocolHead_t;

/* Factory information structure */
typedef struct
{
    uint8_t Software[4];                /* Software version                  */
    uint8_t Hardware[4];                /* Hardware version                  */    
    CreateDate_t Date;                  /* Creation date                     */
    uint8_t SN[24];                     /* Serial number                     */
    uint8_t MAC[4];                     /* MAC address                       */
    uint8_t Res[16];                    /* Reserved                          */
}FactoryInfo_t;

/* Parameter structure */
typedef struct
{
    FactoryInfo_t FactoryInfo;          /* Factory information               */
    uint8_t BaudNum;                    /* Baud rate number                  */
    uint16_t SensorId;                  /* Sensor ID                         */
}Param_t;

extern Param_t Param;                   /* Global parameter structure        */

/* Set data structure */
typedef struct
{
    uint8_t Chn;                        /* Channel number                    */
    float Data;                         /* Data value                        */
}SetData_t;

/* Set ID structure */
typedef struct
{
    uint8_t state;                      /* State                             */
    uint16_t CanId;                     /* CAN ID                            */
}SetId_t;

/* Channel data structure */
typedef struct
{
    uint8_t DataPoint;                  /* Data point                        */
    uint8_t ChannelNum;                 /* Channel number                    */
    int8_t DataPLL;                     /* Data PLL                          */
    uint8_t DataType;                   /* Data type                         */
    float DataValue[];                  /* Flexible array member for values  */
} ChannelData_t;

/* Acknowledge channel data structure */
typedef struct
{
    uint8_t ChannelCnt;                 /* Channel count                     */
    ChannelData_t* channels[];          /* Pointer array for channels        */
} AckChannelData_t;

/* File information structure */
typedef struct
{
    uint8_t FileType;                   /* File type                         */
    p_clock_t FileCreateTime;           /* File creation time                */
    uint32_t FileFlag;                  /* File flag                         */
    uint16_t FileIndex;                 /* File index                        */
    uint32_t FileSize;                  /* File size                         */
    uint8_t res[4];                     /* Reserved                          */
}FileInfo_t;

/* Event information structure */
typedef struct
{
    p_clock_t EventStart;               /* Event start time                  */
    p_clock_t EventEnd;                 /* Event end time                    */
    EVENT_ENUM_T EventType;             /* Event type                        */
    uint8_t EventFrom;                  /* Event source                      */
    uint8_t res;                        /* Reserved                          */
}EventInfo_t;

/* File event information structure */
typedef struct
{
    EventInfo_t EventInfo;              /* Event information                 */
    FileInfo_t FileInfo;                /* File information                  */
}FileEventInfo_t;

/* CAN receive buffer structure */
typedef struct
{
    uint16_t RecvIn;                    /* Receive buffer input index        */
    uint16_t RecvOut;                   /* Receive buffer output index       */
    uint8_t RecvBuf[RX_BUF_LEN];        /* Receive buffer                    */
}CANRxBuf_t;

/* CAN transmit buffer structure */
typedef struct
{
    uint16_t SendId;                    /* Send ID                           */
    uint16_t DataIn;                    /* Transmit buffer input index       */
    uint16_t DataOut;                   /* Transmit buffer output index      */
    uint8_t DataBuf[TX_BUF_LEN];        /* Transmit buffer                   */
}CANTxBuf_t;

/* Function declarations */

/******************************************************************************
 * @brief   Initialize CAN buffers
 * @details Initialize transmit and receive buffers
 * @param   None
 * @return  None
 *****************************************************************************/
void can_buffer_init(void);

/******************************************************************************
 * @brief   Handle CAN transmit operations
 * @details Process transmit buffer and send CAN frames
 * @param   None
 * @return  None
 *****************************************************************************/
void can_tx_handle(void);

/******************************************************************************
 * @brief   Handle CAN receive operations
 * @details Process received CAN frames and store in receive buffer
 * @param   None
 * @return  None
 *****************************************************************************/
void can_rx_handle(void);

/******************************************************************************
 * @brief   Send data via CAN
 * @details Send data with specified command via CAN bus
 * @param   pData Pointer to data buffer
 * @param   Len   Length of data to send
 * @param   Cmd   Command code
 * @return  int8_t Send status
 * @retval  1 Send successful
 * @retval  0 Send failed
 *****************************************************************************/
int8_t api_can_send(uint8_t *pData, uint16_t Len, uint8_t Cmd);

/******************************************************************************
 * @brief   Run CAN processing
 * @details Main CAN processing function, handles received data parsing
 * @param   None
 * @return  None
 *****************************************************************************/
void can_task(void);

#endif /* __CANCMD_H */
