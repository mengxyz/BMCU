#pragma once

#include <stdint.h>

template <typename data_type> class crc
{
public:
    crc(data_type polynomial, data_type init_remainder, data_type final_xor_value);
    void build(data_type polynomial, data_type init_remainder, data_type final_xor_value);

    data_type crc_compute(uint8_t* message, unsigned int nBytes);
    data_type crc_compute(uint8_t* message, unsigned int nBytes, bool reinit);
protected:
    data_type m_polynomial;
    data_type m_initial_remainder;
    data_type m_final_xor_value;
    data_type m_remainder;
    data_type crcTable[256];
    int m_width;
    int m_topbit;
    void crc_init(void);
};

template <typename data_type>

void crc<data_type>::crc_init(void)
{
    data_type remainder;
    int dividend;
    int bit;
    for(dividend = 0; dividend < 256; dividend++)
    {
        remainder = dividend << (m_width - 8);
        for(bit = 0; bit < 8; bit++)
        {
            if(remainder & m_topbit)
            {
                remainder = (remainder << 1) ^ m_polynomial;
            }
            else
            {
                remainder = remainder << 1;
            }
        }
        crcTable[dividend] = remainder;
    }
}

template <typename data_type>
crc<data_type>::crc(data_type polynomial, data_type init_remainder, data_type final_xor_value)
{
    m_width = 8 * sizeof(data_type);
    m_topbit = 1 << (m_width - 1);
    m_polynomial = polynomial;
    m_initial_remainder = init_remainder;
    m_final_xor_value = final_xor_value;

    crc_init();
}

template <typename data_type>
void crc<data_type>::build(data_type polynomial, data_type init_remainder, data_type final_xor_value)
{
    m_polynomial = polynomial;
    m_initial_remainder = init_remainder;
    m_final_xor_value = final_xor_value;

    crc_init();
}

template <typename data_type>
data_type crc<data_type>::crc_compute(uint8_t* message, unsigned int nBytes)
{
    unsigned int offset;
    uint8_t byte;
    data_type remainder = m_initial_remainder;
    /* Divide the message by the polynomial, a byte at a time. */
    for (offset = 0; offset < nBytes; offset++)
    {
        byte = (remainder >> (m_width - 8)) ^ message[offset];
        remainder = crcTable[byte] ^ (remainder << 8);
    }
    /* The final remainder is the CRC result. */
    return (remainder ^ m_final_xor_value);
}

template <typename data_type>
data_type crc<data_type>::crc_compute(uint8_t* message, unsigned int nBytes, bool reinit)
{
    unsigned int offset;
    uint8_t byte;
    if (reinit)
    {
        m_remainder = m_initial_remainder;
    }
    /* Divide the message by the polynomial, a byte at a time. */
    for (offset = 0; offset < nBytes; offset++)
    {
        byte = (m_remainder >> (m_width - 8)) ^ message[offset];
        m_remainder = crcTable[byte] ^ (m_remainder << 8);
    }
    /* The final remainder is the CRC result. */
    return (m_remainder ^ m_final_xor_value);
}

class crc8 : public crc<uint8_t>
{
public:
    crc8(uint8_t polynomial, uint8_t init_remainder, uint8_t final_xor_value)
        :crc<uint8_t>(polynomial, init_remainder, final_xor_value) {}
};

class crc16 : public crc<uint16_t>
{
public:
    crc16(uint16_t polynomial, uint16_t init_remainder, uint16_t final_xor_value)
        :crc<uint16_t>(polynomial, init_remainder, final_xor_value) {}
};

class crc32 : public crc<uint32_t>
{
public:
    crc32(uint32_t polynomial, uint32_t init_remainder, uint32_t final_xor_value)
        :crc<uint32_t>(polynomial, init_remainder, final_xor_value) {}
};

