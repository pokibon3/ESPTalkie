#pragma once
#include <stdlib.h>
#include <stdint.h>

class OutputBuffer;

class Transport
{
protected:
  // audio buffer for samples we need to send
  uint8_t *m_buffer = NULL;
  int m_buffer_size = 0;
  int m_index = 0;
  int m_header_size;

  OutputBuffer *m_output_buffer = NULL;

  virtual void send() = 0;

public:
  Transport(OutputBuffer *output_buffer, size_t buffer_size);
  int set_header(const int header_size, const uint8_t *header);
  void add_sample(int16_t sample);
  void add_sample_u8(uint8_t sample);
  void flush();
  virtual bool        begin() = 0;
  virtual int16_t     getRSSI() = 0;
//  virtual void        setRSSI() = 0;
  virtual uint16_t    getWifiChannel() = 0;
  virtual void        setWifiChannel(uint16_t) = 0;
};
