#include "Detail/Pwm.h"
#include "tim.h"
#include <stdint.h>

enum
{
  PWM_PORT_TIMER_MAX_PERIOD_COUNTS = 65536U,
  PWM_PORT_EXPECTED_COUNTER_FREQUENCY_HZ = 1000000U
};

static bool pwm_port_initialized = false;
static uint32_t pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_COUNT] = {0U};

static uint32_t PwmPort_GetApb1TimerClockFrequencyHz(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  RCC_PeriphCLKInitTypeDef peripheral_clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t timer_clock_frequency_hz;

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  HAL_RCCEx_GetPeriphCLKConfig(&peripheral_clock_config);

  if (peripheral_clock_config.TIMPresSelection == RCC_TIMPRES_DESACTIVATED)
  {
    if (clock_config.APB1CLKDivider == RCC_APB1_DIV1 || clock_config.APB1CLKDivider == RCC_APB1_DIV2)
    {
      return HAL_RCC_GetHCLKFreq();
    }

    return HAL_RCC_GetPCLK1Freq() * 2U;
  }

  if (clock_config.APB1CLKDivider == RCC_APB1_DIV1 || clock_config.APB1CLKDivider == RCC_APB1_DIV2 ||
      clock_config.APB1CLKDivider == RCC_APB1_DIV4)
  {
    return HAL_RCC_GetHCLKFreq();
  }

  timer_clock_frequency_hz = HAL_RCC_GetPCLK1Freq();
  return timer_clock_frequency_hz * 4U;
}

static TIM_HandleTypeDef *PwmPort_GetTimer(PwmPort_Channel channel)
{
  switch (channel)
  {
  case PWM_PORT_CHANNEL_BUZZER:
    return &htim12;

  case PWM_PORT_CHANNEL_IMU_HEATER:
    return &htim3;

  default:
    return NULL;
  }
}

static uint32_t PwmPort_GetTimerChannel(PwmPort_Channel channel)
{
  return channel == PWM_PORT_CHANNEL_IMU_HEATER ? TIM_CHANNEL_4 : TIM_CHANNEL_2;
}

bool PwmPort_Init(void)
{
  uint32_t timer_clock_frequency_hz;
  uint32_t channel;

  pwm_port_initialized = false;
  for (channel = 0U; channel < PWM_PORT_CHANNEL_COUNT; ++channel)
  {
    pwm_port_counter_frequency_hz[channel] = 0U;
  }

  if (htim12.Instance != TIM12 || htim12.State != HAL_TIM_STATE_READY || htim3.Instance != TIM3 ||
      htim3.State != HAL_TIM_STATE_READY)
  {
    return false;
  }

  timer_clock_frequency_hz = PwmPort_GetApb1TimerClockFrequencyHz();
  if (timer_clock_frequency_hz == 0U || htim12.Init.Prescaler == UINT32_MAX || htim3.Init.Prescaler == UINT32_MAX)
  {
    return false;
  }

  pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_BUZZER] = timer_clock_frequency_hz / (htim12.Init.Prescaler + 1U);
  pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_IMU_HEATER] = timer_clock_frequency_hz / (htim3.Init.Prescaler + 1U);
  if (pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_BUZZER] != PWM_PORT_EXPECTED_COUNTER_FREQUENCY_HZ ||
      pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_IMU_HEATER] != PWM_PORT_EXPECTED_COUNTER_FREQUENCY_HZ)
  {
    pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_BUZZER] = 0U;
    pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_IMU_HEATER] = 0U;
    return false;
  }

  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK)
  {
    pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_BUZZER] = 0U;
    pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_IMU_HEATER] = 0U;
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_BUZZER] = 0U;
    pwm_port_counter_frequency_hz[PWM_PORT_CHANNEL_IMU_HEATER] = 0U;
    return false;
  }

  pwm_port_initialized = true;
  return true;
}

bool PwmPort_IsReady(PwmPort_Channel channel)
{
  return pwm_port_initialized && channel < PWM_PORT_CHANNEL_COUNT;
}

PwmPort_Result PwmPort_Set(PwmPort_Channel channel, uint32_t frequency_hz, uint16_t duty_permille)
{
  TIM_HandleTypeDef *timer;
  uint32_t timer_channel;
  uint32_t period_counts;
  uint32_t compare_counts;

  if (channel >= PWM_PORT_CHANNEL_COUNT || frequency_hz == 0U || duty_permille > PWM_PORT_DUTY_PERMILLE_MAX)
  {
    return PWM_PORT_RESULT_INVALID_ARGUMENT;
  }

  if (!PwmPort_IsReady(channel))
  {
    return PWM_PORT_RESULT_NOT_READY;
  }

  timer = PwmPort_GetTimer(channel);
  timer_channel = PwmPort_GetTimerChannel(channel);
  if (timer == NULL)
  {
    return PWM_PORT_RESULT_INVALID_ARGUMENT;
  }

  period_counts = (uint32_t)(((uint64_t)pwm_port_counter_frequency_hz[channel] + frequency_hz / 2U) / frequency_hz);
  if (period_counts < 2U || period_counts > PWM_PORT_TIMER_MAX_PERIOD_COUNTS)
  {
    return PWM_PORT_RESULT_INVALID_ARGUMENT;
  }

  compare_counts =
      (period_counts * (uint32_t)duty_permille + PWM_PORT_DUTY_PERMILLE_MAX / 2U) / PWM_PORT_DUTY_PERMILLE_MAX;

  __HAL_TIM_SET_COMPARE(timer, timer_channel, 0U);
  __HAL_TIM_SET_AUTORELOAD(timer, period_counts - 1U);
  __HAL_TIM_SET_COUNTER(timer, 0U);
  __HAL_TIM_SET_COMPARE(timer, timer_channel, compare_counts);
  return PWM_PORT_RESULT_COMPLETED;
}

PwmPort_Result PwmPort_Silence(PwmPort_Channel channel)
{
  TIM_HandleTypeDef *timer;

  if (channel >= PWM_PORT_CHANNEL_COUNT)
  {
    return PWM_PORT_RESULT_INVALID_ARGUMENT;
  }

  if (!PwmPort_IsReady(channel))
  {
    return PWM_PORT_RESULT_NOT_READY;
  }

  timer = PwmPort_GetTimer(channel);
  if (timer == NULL)
  {
    return PWM_PORT_RESULT_INVALID_ARGUMENT;
  }

  __HAL_TIM_SET_COMPARE(timer, PwmPort_GetTimerChannel(channel), 0U);
  return PWM_PORT_RESULT_COMPLETED;
}
