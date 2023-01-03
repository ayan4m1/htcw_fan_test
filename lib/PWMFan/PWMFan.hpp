#ifndef PWM_FAN_HPP
#define PWM_FAN_HPP

#include <Arduino.h>
#include <PIDAutotuner.h>
#include <pid.h>

#define FAN_LEDC_CHANNEL 0
#define FAN_PWM_RESOLUTION_BITS 4   // 16 speeds
#define FAN_PWM_FREQUENCY_HZ 2.5e4  // 25kHz
#define FAN_SPEED_MAX 0b1111
#define FAN_SPEED_MIN 0b0000

struct PIDs {
  double Kp = 0, Ki = 0, Kd = 0;
};

class PWMFan final {
  uint32_t last_update;
  volatile uint32_t ticks;
  uint8_t tach_pin;
  uint8_t pwm_pin;
  uint8_t min_rpm;
  uint8_t hz_to_rpm;
  uint16_t target_rpm = 0;
  uint16_t current_rpm = 0;
  uint32_t tach_timeout = 0;
  PIDs pids = PIDs();
  epid_t pid;
  PIDAutotuner tuner = PIDAutotuner();
  static void IRAM_ATTR interrupt(void* state);

 public:
  PWMFan(const uint8_t tach_pin, const uint8_t pwm_pin,
         const uint16_t min_rpm = 0, const uint8_t hz_to_rpm = 30);
  PIDs tune(double max_rpm, uint32_t loop_interval_us);
  void begin();
  void end();
  void update();
  void setTargetRpm(const uint16_t target_rpm);
  bool setPIDs(const PIDs pids);
  uint16_t getTargetRpm();
  uint16_t getCurrentRpm();
};

#endif
