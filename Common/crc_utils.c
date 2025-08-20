/**
 * @file crc_utils.c
 * @brief 通用CRC校验算法实现
 */

#include "crc_utils.h"
#include <string.h>

// 预定义的多项式和参数
static const struct {
    uint32_t polynomial;
    uint32_t initial_value;
    uint32_t final_xor;
    bool reflect_input;
    bool reflect_output;
} crc_params[] = {
    // CRC_TYPE_8_CCITT
    {0x07, 0x00, 0x00, false, false},
    // CRC_TYPE_8_MAXIM
    {0x31, 0x00, 0x00, true, true},
    // CRC_TYPE_16_CCITT
    {0x1021, 0x0000, 0x0000, false, false},
    // CRC_TYPE_16_MODBUS
    {0x8005, 0xFFFF, 0x0000, true, true},
    // CRC_TYPE_32_IEEE
    {0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true}
};

bool crc_init_context(crc_context_t* ctx, crc_type_t type)
{
    if (!ctx || type >= CRC_TYPE_MAX) {
        return false;
    }
    
    ctx->type = type;
    ctx->polynomial = crc_params[type].polynomial;
    ctx->initial_value = crc_params[type].initial_value;
    ctx->final_xor = crc_params[type].final_xor;
    ctx->reflect_input = crc_params[type].reflect_input;
    ctx->reflect_output = crc_params[type].reflect_output;
    ctx->current_crc = ctx->initial_value;
    
    return true;
}

// 位反转辅助函数
static uint8_t reflect8(uint8_t data)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            result |= (1 << (7 - i));
        }
    }
    return result;
}

static uint16_t reflect16(uint16_t data)
{
    uint16_t result = 0;
    for (int i = 0; i < 16; i++) {
        if (data & (1 << i)) {
            result |= (1 << (15 - i));
        }
    }
    return result;
}

static uint32_t reflect32(uint32_t data)
{
    uint32_t result = 0;
    for (int i = 0; i < 32; i++) {
        if (data & (1U << i)) {
            result |= (1U << (31 - i));
        }
    }
    return result;
}

// CRC-8计算
uint8_t crc8_calculate(const uint8_t* data, size_t length, crc_type_t type)
{
    if (!data || type >= CRC_TYPE_MAX || type > CRC_TYPE_8_MAXIM) {
        return 0;
    }
    
    const uint8_t polynomial = (uint8_t)crc_params[type].polynomial;
    uint8_t crc = (uint8_t)crc_params[type].initial_value;
    bool reflect_input = crc_params[type].reflect_input;
    bool reflect_output = crc_params[type].reflect_output;
    uint8_t final_xor = (uint8_t)crc_params[type].final_xor;
    
    for (size_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        if (reflect_input) {
            byte = reflect8(byte);
        }
        
        crc ^= byte;
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    
    if (reflect_output) {
        crc = reflect8(crc);
    }
    
    return crc ^ final_xor;
}

// CRC-16计算
uint16_t crc16_calculate(const uint8_t* data, size_t length, crc_type_t type)
{
    if (!data || type >= CRC_TYPE_MAX || type < CRC_TYPE_16_CCITT) {
        return 0;
    }
    
    if (type > CRC_TYPE_16_MODBUS) {
        return 0;
    }
    
    const uint16_t polynomial = (uint16_t)crc_params[type].polynomial;
    uint16_t crc = (uint16_t)crc_params[type].initial_value;
    bool reflect_input = crc_params[type].reflect_input;
    bool reflect_output = crc_params[type].reflect_output;
    uint16_t final_xor = (uint16_t)crc_params[type].final_xor;
    
    for (size_t i = 0; i < length; i++) {
        uint16_t byte = data[i];
        if (reflect_input) {
            byte = reflect8((uint8_t)byte);
        }
        
        crc ^= (byte << 8);
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    
    if (reflect_output) {
        crc = reflect16(crc);
    }
    
    return crc ^ final_xor;
}

// CRC-32计算
uint32_t crc32_calculate(const uint8_t* data, size_t length, crc_type_t type)
{
    if (!data || type != CRC_TYPE_32_IEEE) {
        return 0;
    }
    
    const uint32_t polynomial = crc_params[type].polynomial;
    uint32_t crc = crc_params[type].initial_value;
    bool reflect_input = crc_params[type].reflect_input;
    bool reflect_output = crc_params[type].reflect_output;
    uint32_t final_xor = crc_params[type].final_xor;
    
    for (size_t i = 0; i < length; i++) {
        uint32_t byte = data[i];
        if (reflect_input) {
            byte = reflect8((uint8_t)byte);
        }
        
        crc ^= (byte << 24);
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    
    if (reflect_output) {
        crc = reflect32(crc);
    }
    
    return crc ^ final_xor;
}

// 增量计算接口实现
void crc_context_begin(crc_context_t* ctx)
{
    if (ctx) {
        ctx->current_crc = ctx->initial_value;
    }
}

void crc_context_update(crc_context_t* ctx, const uint8_t* data, size_t length)
{
    if (!ctx || !data) {
        return;
    }
    
    // 根据CRC类型选择相应的计算方法
    if (ctx->type <= CRC_TYPE_8_MAXIM) {
        // CRC-8
        for (size_t i = 0; i < length; i++) {
            uint8_t byte = data[i];
            if (ctx->reflect_input) {
                byte = reflect8(byte);
            }
            
            ctx->current_crc ^= byte;
            
            for (int j = 0; j < 8; j++) {
                if (ctx->current_crc & 0x80) {
                    ctx->current_crc = (ctx->current_crc << 1) ^ (uint8_t)ctx->polynomial;
                } else {
                    ctx->current_crc <<= 1;
                }
            }
        }
    } else if (ctx->type <= CRC_TYPE_16_MODBUS) {
        // CRC-16
        for (size_t i = 0; i < length; i++) {
            uint16_t byte = data[i];
            if (ctx->reflect_input) {
                byte = reflect8((uint8_t)byte);
            }
            
            ctx->current_crc ^= (byte << 8);
            
            for (int j = 0; j < 8; j++) {
                if (ctx->current_crc & 0x8000) {
                    ctx->current_crc = (ctx->current_crc << 1) ^ (uint16_t)ctx->polynomial;
                } else {
                    ctx->current_crc <<= 1;
                }
            }
        }
    } else if (ctx->type == CRC_TYPE_32_IEEE) {
        // CRC-32
        for (size_t i = 0; i < length; i++) {
            uint32_t byte = data[i];
            if (ctx->reflect_input) {
                byte = reflect8((uint8_t)byte);
            }
            
            ctx->current_crc ^= (byte << 24);
            
            for (int j = 0; j < 8; j++) {
                if (ctx->current_crc & 0x80000000) {
                    ctx->current_crc = (ctx->current_crc << 1) ^ ctx->polynomial;
                } else {
                    ctx->current_crc <<= 1;
                }
            }
        }
    }
}

uint32_t crc_context_finish(crc_context_t* ctx)
{
    if (!ctx) {
        return 0;
    }
    
    uint32_t result = ctx->current_crc;
    
    if (ctx->reflect_output) {
        if (ctx->type <= CRC_TYPE_8_MAXIM) {
            result = reflect8((uint8_t)result);
        } else if (ctx->type <= CRC_TYPE_16_MODBUS) {
            result = reflect16((uint16_t)result);
        } else {
            result = reflect32(result);
        }
    }
    
    return result ^ ctx->final_xor;
}

// 查表计算接口实现
uint8_t crc8_table_calculate(const uint8_t* data, size_t length, const uint8_t* table)
{
    if (!data || !table) {
        return 0;
    }
    
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = table[crc ^ data[i]];
    }
    return crc;
}

uint16_t crc16_table_calculate(const uint8_t* data, size_t length, const uint16_t* table)
{
    if (!data || !table) {
        return 0;
    }
    
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        uint8_t index = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (crc << 8) ^ table[index];
    }
    return crc;
}

uint32_t crc32_table_calculate(const uint8_t* data, size_t length, const uint32_t* table)
{
    if (!data || !table) {
        return 0;
    }
    
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        uint8_t index = (uint8_t)((crc ^ data[i]) & 0xFF);
        crc = (crc >> 8) ^ table[index];
    }
    return crc ^ 0xFFFFFFFF;
}

// 表生成函数实现
void crc8_generate_table(uint8_t* table, uint8_t polynomial)
{
    if (!table) {
        return;
    }
    
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
        table[i] = crc;
    }
}

void crc16_generate_table(uint16_t* table, uint16_t polynomial)
{
    if (!table) {
        return;
    }
    
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
        table[i] = crc;
    }
}

void crc32_generate_table(uint32_t* table, uint32_t polynomial)
{
    if (!table) {
        return;
    }
    
    for (int i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
        table[i] = crc;
    }
}
