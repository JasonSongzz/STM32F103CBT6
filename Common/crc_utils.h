/**
 * @file crc_utils.h
 * @brief 通用CRC校验算法模块
 */

#ifndef CRC_UTILS_H
#define CRC_UTILS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// CRC类型定义
typedef enum {
    CRC_TYPE_8_CCITT,      // CRC-8/CCITT
    CRC_TYPE_8_MAXIM,      // CRC-8/MAXIM
    CRC_TYPE_16_CCITT,     // CRC-16/CCITT
    CRC_TYPE_16_MODBUS,    // CRC-16/MODBUS
    CRC_TYPE_32_IEEE,      // CRC-32/IEEE802.3
    CRC_TYPE_MAX
} crc_type_t;

// CRC计算上下文
typedef struct {
    crc_type_t type;
    uint32_t polynomial;
    uint32_t initial_value;
    uint32_t final_xor;
    bool reflect_input;
    bool reflect_output;
    uint32_t current_crc;
} crc_context_t;

// 函数声明
bool crc_init_context(crc_context_t* ctx, crc_type_t type);
uint8_t crc8_calculate(const uint8_t* data, size_t length, crc_type_t type);
uint16_t crc16_calculate(const uint8_t* data, size_t length, crc_type_t type);
uint32_t crc32_calculate(const uint8_t* data, size_t length, crc_type_t type);

// 增量计算接口
void crc_context_begin(crc_context_t* ctx);
void crc_context_update(crc_context_t* ctx, const uint8_t* data, size_t length);
uint32_t crc_context_finish(crc_context_t* ctx);

// 直接查表计算接口
uint8_t crc8_table_calculate(const uint8_t* data, size_t length, const uint8_t* table);
uint16_t crc16_table_calculate(const uint8_t* data, size_t length, const uint16_t* table);
uint32_t crc32_table_calculate(const uint8_t* data, size_t length, const uint32_t* table);

// 表生成函数
void crc8_generate_table(uint8_t* table, uint8_t polynomial);
void crc16_generate_table(uint16_t* table, uint16_t polynomial);
void crc32_generate_table(uint32_t* table, uint32_t polynomial);

#endif // CRC_UTILS_H
