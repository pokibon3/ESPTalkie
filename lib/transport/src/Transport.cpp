#include "Arduino.h"
#include "Transport.h"

Transport::Transport(OutputBuffer *output_buffer, size_t buffer_size)
{
  m_output_buffer = output_buffer;
    m_buffer_size = buffer_size;
    m_buffer = (uint8_t *)malloc(m_buffer_size);
    m_index = 0;
    m_header_size = 0;
}

void Transport::add_sample(int16_t sample)
{
    static bool gate_open = false;
    static int32_t hold = 0;
    constexpr int32_t gate_open_th = 520;
    constexpr int32_t gate_close_th = 360;
    constexpr int32_t gate_hold_samples = 192;  // ~12ms @16k

    int32_t x = sample;
    int32_t level = (x >= 0) ? x : -x;

    if (gate_open) {
        if (level < gate_close_th) {
            if (hold > 0) {
                --hold;
            } else {
                gate_open = false;
            }
        } else {
            hold = gate_hold_samples;
        }
    } else {
        if (level > gate_open_th) {
            gate_open = true;
            hold = gate_hold_samples;
        }
    }

    if (!gate_open) {
        x = 0;
    }

    int16_t work = static_cast<int16_t>(x >> 3);
    work = (work < -128) ? -128 : ((work > 127) ? 127 : work);
    uint8_t work2 = (work + 128) & 0x00ff;
    add_sample_u8(work2);
}

void Transport::add_sample_u8(uint8_t sample)
{
    m_buffer[m_index+m_header_size] = sample;
    m_index++;
    // have we reached a full packet?
    if ((m_index + m_header_size) == m_buffer_size) {
        send();
        m_index = 0;
    }
}

void Transport::flush()
{
    if (m_index >0 ) {
        send();
        m_index = 0;
    }
}

int Transport::set_header(const int header_size, const uint8_t *header)
{
    if ((header_size<m_buffer_size) && (header)) {
        m_header_size = header_size;
        memcpy(m_buffer, header, header_size);
        return 0;
    } else {
        return -1;
    }
}
