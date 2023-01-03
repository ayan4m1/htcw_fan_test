#include "PWMFan.hpp"

PWMFan::PWMFan(const uint8_t tach_pin, const uint8_t pwm_pin,
               const uint16_t min_rpm, const uint8_t hz_to_rpm) {
  this->tach_pin = tach_pin;
  this->pwm_pin = pwm_pin;
  this->min_rpm = min_rpm;
  this->hz_to_rpm = hz_to_rpm;
  this->tach_timeout = hz_to_rpm > 0 ? (1 / (min_rpm / hz_to_rpm)) * 1e6 : 0;
  this->ticks = 0;
  this->last_update = micros();
}

void PWMFan::interrupt(void* state) {
  auto fan = (PWMFan*)state;
  ++fan->ticks;
}

void PWMFan::begin() {
  pinMode(tach_pin, INPUT);
  pinMode(pwm_pin, OUTPUT);

  ledcSetup(FAN_LEDC_CHANNEL, FAN_PWM_FREQUENCY_HZ, FAN_PWM_RESOLUTION_BITS);
  ledcAttachPin(pwm_pin, FAN_LEDC_CHANNEL);
  ledcWrite(FAN_LEDC_CHANNEL, 0);

  ticks = 0;
  last_update = micros();
  attachInterruptArg(tach_pin, interrupt, this, RISING);
}

void PWMFan::end() {
  ledcDetachPin(pwm_pin);
  detachInterrupt(tach_pin);
}

void PWMFan::update() {
  auto time = micros();
  auto elapsed_us = time - last_update;
  auto elapsed_seconds = elapsed_us / (float)1e6;
  auto hz = ticks / elapsed_seconds;

  current_rpm = hz * hz_to_rpm;
  last_update = micros();
  ticks = 0;

  if (pids.Kp == 0) {
    ledcWrite(FAN_LEDC_CHANNEL, FAN_SPEED_MAX);
    return;
  }

  epid_pid_calc(&pid, target_rpm, current_rpm);
  epid_pid_sum(&pid, FAN_SPEED_MIN, FAN_SPEED_MAX);

  ledcWrite(FAN_LEDC_CHANNEL, (uint32_t)lroundf(pid.y_out));
}

uint16_t PWMFan::getCurrentRpm() { return current_rpm; }

uint16_t PWMFan::getTargetRpm() { return target_rpm; }

void PWMFan::setTargetRpm(const uint16_t target_rpm) {
  this->target_rpm = target_rpm;
}

bool PWMFan::setPIDs(const PIDs pids) {
  this->pids.Kp = pids.Kp;
  this->pids.Ki = pids.Ki;
  this->pids.Kd = pids.Kd;

  auto epid_err = epid_init(&pid, current_rpm, current_rpm, FAN_SPEED_MIN,
                            pids.Kp, pids.Ki, pids.Kd);

  return epid_err == EPID_ERR_NONE;
}

PIDs PWMFan::tune(double max_rpm, uint32_t loop_interval_us) {
  tuner.setTargetInputValue(max_rpm);
  tuner.setLoopInterval(loop_interval_us);
  tuner.setOutputRange(FAN_SPEED_MIN, FAN_SPEED_MAX);
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

  ledcWrite(FAN_LEDC_CHANNEL, FAN_SPEED_MAX);

  uint32_t us = micros();
  tuner.startTuningLoop(us);
  while (!tuner.isFinished()) {
    us = micros();

    update();
    auto output = tuner.tunePID(current_rpm, us);
    ledcWrite(FAN_LEDC_CHANNEL, output);

    while (micros() - us < loop_interval_us) {
      delayMicroseconds(1);
    }
  }

  ledcWrite(FAN_LEDC_CHANNEL, 0);

  Serial.printf("Kp: %.2f\n", tuner.getKp());
  Serial.printf("Ki: %.2f\n", tuner.getKi());
  Serial.printf("Kd: %.2f\n", tuner.getKd());

  auto tune = PIDs();

  tune.Kp = tuner.getKp();
  tune.Ki = tuner.getKi();
  tune.Kd = tuner.getKd();

  return tune;
}
