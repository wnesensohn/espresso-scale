#ifndef FlowMeter_h
#define FlowMeter_h

#include <inttypes.h>

class FlowMeter
{

public:
  FlowMeter()
  {
    _buffer_ptr = 255;
    _window_size = 255;

    // Init buffers
    clear();
  };

  // clear the filter, setting all values to the supplied value
  void clear()
  {
    uint8_t i = _window_size;
    while(i > 0)
    {
      i--;
      _timestamps[i] = 0;
      _weights[i] = 0.0f;
    }
  }

  float getCurrentFlow()
  {
    uint8_t current_ptr = _buffer_ptr;
    uint8_t last_ptr = _buffer_ptr + 10;

    unsigned long millis_diff = _timestamps[current_ptr] - _timestamps[last_ptr];
    float weight_diff = _weights[current_ptr] - _weights[last_ptr];

    if(millis_diff == 0)
    {
      // TODO: use an even older value here, if possible
      return 0;
    }

    return weight_diff * 1000.0 / millis_diff;
  }

  void addValue(unsigned long time, float value)
  {
    _buffer_ptr--;

    _timestamps[_buffer_ptr] = time;
    _weights[_buffer_ptr] = value;

  }

private:
  // Pointer to the last added element in buffers
  uint8_t _buffer_ptr;
  // sliding window size
  uint8_t _window_size;

  // cyclic buffer for incoming values
  unsigned long _timestamps[256];
  // sorted buffer
  float _weights[256];
};

#endif
