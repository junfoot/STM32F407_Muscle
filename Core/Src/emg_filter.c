/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    emg_filter.c
  * @brief   Real-time 16-channel sEMG filter.
  *
  * Filter chain at Fs = 2000 Hz:
  *   1. 2nd-order Butterworth high-pass, Fc = 20 Hz
  *   2. 50 Hz mains notch, Q = 30
  *   3. 2nd-order Butterworth low-pass, Fc = 450 Hz
  *
  * The implementation uses one independent state chain per channel.  It is
  * called once for every completed AD7606 conversion, so decimation of the
  * USART stream does not reduce the filter sample rate.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "emg_filter.h"

#define EMG_ADC_LSB_VOLTS  (5.0f / 32768.0f)  /* AD7606 +/-5 V range */
#define EMG_FILTER_STAGES  3u

typedef struct
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float x1;
  float x2;
  float y1;
  float y2;
} EMG_Biquad_t;

/* Coefficients are normalized by a0 and calculated for Fs = 2000 Hz. */
static const float emg_coefficients[EMG_FILTER_STAGES][5] =
{
  /* b0,          b1,           b2,          a1,           a2 */
  { 0.956543226f, -1.913086451f, 0.956543226f, -1.911197067f, 0.914975835f }, /* HP 20 Hz */
  { 0.997399539f, -1.970239791f, 0.997399539f, -1.970239791f, 0.994799078f }, /* Notch 50 Hz */
  { 0.248341079f,  0.496682158f, 0.248341079f, -0.184213803f, 0.177578119f }  /* LP 450 Hz */
};

static EMG_Biquad_t emg_state[EMG_FILTER_CHANNELS][EMG_FILTER_STAGES];
static uint8_t emg_initialized;

static float emg_biquad_process(EMG_Biquad_t *state, float input)
{
  float output;

  output = (state->b0 * input) + (state->b1 * state->x1) +
           (state->b2 * state->x2) - (state->a1 * state->y1) -
           (state->a2 * state->y2);

  state->x2 = state->x1;
  state->x1 = input;
  state->y2 = state->y1;
  state->y1 = output;

  return output;
}

void EMG_Filter_Init(void)
{
  uint32_t channel;
  uint32_t stage;

  for (channel = 0u; channel < EMG_FILTER_CHANNELS; channel++)
  {
    for (stage = 0u; stage < EMG_FILTER_STAGES; stage++)
    {
      EMG_Biquad_t *state = &emg_state[channel][stage];

      state->b0 = emg_coefficients[stage][0];
      state->b1 = emg_coefficients[stage][1];
      state->b2 = emg_coefficients[stage][2];
      state->a1 = emg_coefficients[stage][3];
      state->a2 = emg_coefficients[stage][4];
      state->x1 = 0.0f;
      state->x2 = 0.0f;
      state->y1 = 0.0f;
      state->y2 = 0.0f;
    }
  }

  emg_initialized = 0u;
}

void EMG_Filter_Process(const int16_t *raw_adc, volatile float *filtered_volts)
{
  uint32_t channel;

  /* Seed each section with the first sample to avoid a large power-up
     transient from the ADC's common-mode/DC level. */
  if (emg_initialized == 0u)
  {
    for (channel = 0u; channel < EMG_FILTER_CHANNELS; channel++)
    {
      float input = (float)raw_adc[channel] * EMG_ADC_LSB_VOLTS;

      emg_state[channel][0].x1 = input;
      emg_state[channel][0].x2 = input;
      emg_state[channel][0].y1 = 0.0f; /* HP DC gain = 0 */
      emg_state[channel][0].y2 = 0.0f;

      emg_state[channel][1].x1 = 0.0f;
      emg_state[channel][1].x2 = 0.0f;
      emg_state[channel][1].y1 = 0.0f;
      emg_state[channel][1].y2 = 0.0f;

      emg_state[channel][2].x1 = 0.0f;
      emg_state[channel][2].x2 = 0.0f;
      emg_state[channel][2].y1 = 0.0f;
      emg_state[channel][2].y2 = 0.0f;
    }
    emg_initialized = 1u;
  }

  for (channel = 0u; channel < EMG_FILTER_CHANNELS; channel++)
  {
    float sample = (float)raw_adc[channel] * EMG_ADC_LSB_VOLTS;

    sample = emg_biquad_process(&emg_state[channel][0], sample);
    sample = emg_biquad_process(&emg_state[channel][1], sample);
    sample = emg_biquad_process(&emg_state[channel][2], sample);
    filtered_volts[channel] = sample;
  }
}
