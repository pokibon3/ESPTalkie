#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

/**
 * @brief Circular buffer for 8 bit unsigned PCM samples
 * 
 */
class OutputBuffer
{
private:
  // how many samples should we buffer before outputting data?
  int m_number_samples_to_buffer;
  // where are we reading from
  int m_read_head;
  // where are we writing to
  int m_write_head;
  // keep track of how many samples we have
  int m_available_samples;
  // the total size of the buffer
  int m_buffer_size;
  // are we currently buffering samples?
  bool m_buffering;
  // diagnostics
  uint32_t m_underrun_events;
  uint32_t m_overflow_events;
  // last emitted sample for smooth concealment / recovery
  uint8_t m_last_output_sample;
  int m_recover_samples;
  // the sample buffer
  uint8_t *m_buffer;
  // thread safety
  SemaphoreHandle_t m_semaphore;

public:
  OutputBuffer(int number_samples_to_buffer) : m_number_samples_to_buffer(number_samples_to_buffer)
  {
    // create a semaphore and make it available for locking
    m_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(m_semaphore);
    // set reading and writing to the beginning of the buffer
    m_read_head = 0;
    m_write_head = 0;
    m_available_samples = 0;
    // we'll start off buffering data as we have no samples yet
    m_buffering = true;
    m_underrun_events = 0;
    m_overflow_events = 0;
    m_last_output_sample = 128;
    m_recover_samples = 0;
    // make sufficient space for the bufferring and incoming data
    m_buffer_size = 3 * number_samples_to_buffer;
    m_buffer = (uint8_t *)malloc(m_buffer_size);
    memset(m_buffer, 0, m_buffer_size);
    if (!m_buffer)
    {
      Serial.println("Failed to allocate buffer");
    }
  }

  // we're adding samples that are 8 bit as they are coming from the transport
  void add_samples(const uint8_t *samples, int count)
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    // copy the samples into the buffer wrapping around as needed
    for (int i = 0; i < count; i++)
    {
      m_buffer[m_write_head] = samples[i];
      m_write_head = (m_write_head + 1) % m_buffer_size;
      if (m_available_samples < m_buffer_size) {
        m_available_samples++;
      } else {
        // drop the oldest sample on overflow to keep buffer state consistent
        m_read_head = (m_read_head + 1) % m_buffer_size;
        ++m_overflow_events;
      }
    }
    xSemaphoreGive(m_semaphore);
  }

  // convert the samples to 16 bit as they are going to the output
//void remove_samples(uint16_t *samples, int count)
  void remove_samples(uint8_t *samples, int count)
  {
    //Serial.println("remove samples");
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    for (int i = 0; i < count; i++)
    {
      samples[i] = m_last_output_sample;
      // if we have no samples and we aren't already buffering then we need to start buffering
      if (m_available_samples == 0 && !m_buffering)
      {
        m_buffering = true;
        ++m_underrun_events;
        m_recover_samples = 32;
      }
      // are we buffering?
      if (m_buffering && m_available_samples < m_number_samples_to_buffer)
      {
        // conceal gap smoothly by easing toward center instead of hard 128.
        int s = static_cast<int>(m_last_output_sample);
        int d = 128 - s;
        if (d > 4) d = 4;
        if (d < -4) d = -4;
        s += d;
        if (s < 0) s = 0;
        if (s > 255) s = 255;
        samples[i] = static_cast<uint8_t>(s);
        m_last_output_sample = samples[i];
      }
      else
      {
        // we've buffered enough samples so no need to buffer anymore
        if (m_buffering) {
          m_buffering = false;
          m_recover_samples = 32;
        }
        // send buffered sample and move the read head forward
        uint8_t raw = m_buffer[m_read_head];
        m_read_head = (m_read_head + 1) % m_buffer_size;
        m_available_samples--;

        int out = static_cast<int>(raw);
        if (m_recover_samples > 0) {
          // Slew-limit immediately after recovery to avoid sharp click.
          const int prev = static_cast<int>(m_last_output_sample);
          int diff = out - prev;
          const int kMaxStep = 12;
          if (diff > kMaxStep) out = prev + kMaxStep;
          if (diff < -kMaxStep) out = prev - kMaxStep;
          --m_recover_samples;
        }
        if (out < 0) out = 0;
        if (out > 255) out = 255;
        samples[i] = static_cast<uint8_t>(out);
        m_last_output_sample = samples[i];
      }
    }
    xSemaphoreGive(m_semaphore);
  }

  int get_available_samples()
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    int v = m_available_samples;
    xSemaphoreGive(m_semaphore);
    return v;
  }

  int get_buffer_size()
  {
    return m_buffer_size;
  }

  int get_target_buffer_samples()
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    int v = m_number_samples_to_buffer;
    xSemaphoreGive(m_semaphore);
    return v;
  }

  void set_target_buffer_samples(int target_samples)
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    if (target_samples < 1) {
      target_samples = 1;
    }
    if (target_samples >= m_buffer_size) {
      target_samples = m_buffer_size - 1;
    }
    m_number_samples_to_buffer = target_samples;
    xSemaphoreGive(m_semaphore);
  }

  void snapshot_and_reset_stats(uint32_t &underruns, uint32_t &overflows)
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    underruns = m_underrun_events;
    overflows = m_overflow_events;
    m_underrun_events = 0;
    m_overflow_events = 0;
    xSemaphoreGive(m_semaphore);
  }
};
