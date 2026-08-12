#pragma once

#include "esphome/core/component.h"
#include "esphome/core/optional.h"
#include "esphome/core/time.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/template/sensor/template_sensor.h"
#include "esphome/components/template/text_sensor/template_text_sensor.h"

#include <map>
#include <vector>
#include <string>

namespace esphome
{
  namespace mower
  {

    class Automower : public PollingComponent, public uart::UARTDevice
    {
    public:
      Automower(uart::UARTComponent *parent, uint32_t update_interval);

      void set_battery_level_sensor(template_::TemplateSensor *s);
      void set_battery_temperature_sensor(template_::TemplateSensor *s);
      void set_battery_used_sensor(template_::TemplateSensor *s);
      void set_battery_voltage_sensor(template_::TemplateSensor *s);
      void set_blade_motor_speed_sensor(template_::TemplateSensor *s);
      void set_charging_time_sensor(template_::TemplateSensor *s);
      void set_mowing_time_sensor(template_::TemplateSensor *s);
      void set_firmware_version_sensor(template_::TemplateSensor *s);

      void set_last_code_received_text_sensor(template_::TemplateTextSensor *s);
      void set_mode_text_sensor(template_::TemplateTextSensor *s);
      void set_status_text_sensor(template_::TemplateTextSensor *s);

      template_::TemplateSensor *get_battery_level_sensor() const;
      template_::TemplateSensor *get_battery_temperature_sensor() const;
      template_::TemplateSensor *get_battery_used_sensor() const;
      template_::TemplateSensor *get_battery_voltage_sensor() const;
      template_::TemplateSensor *get_blade_motor_speed_sensor() const;
      template_::TemplateSensor *get_charging_time_sensor() const;
      template_::TemplateSensor *get_mowing_time_sensor() const;
      template_::TemplateSensor *get_firmware_version_sensor() const;

      template_::TemplateTextSensor *get_last_code_received_text_sensor() const;
      template_::TemplateTextSensor *get_mode_text_sensor() const;
      template_::TemplateTextSensor *get_status_text_sensor() const;

      // Generic access to the last received value of any register, for the
      // template sensors/numbers defined in yaml. Returns NaN when the register
      // hasn't been read yet, so the lambda can skip publishing (return {}).
      float get_register(uint16_t addr, bool is_signed = false);
      // Timer registers live at 0x4A00 | reg; pass the low reg bytes (e.g. 0x38).
      esphome::optional<esphome::ESPTime> get_timer_time(uint8_t hour_reg, uint8_t minute_reg);
      esphome::optional<bool> get_timer_active();

      void setup() override;
      void update() override;
      void loop() override;

      void set_mode(const std::string &value);
      void set_stop(bool stop);
      void set_left_motor(int value);
      void set_right_motor(int value);
      void key_back();
      void key_yes();
      void key_num(uint8_t num);

      // Write a single timer register (0x4A00 | reg is the read address; the
      // write uses the same low byte with the high bit set: 0xCA). value is the
      // hour (0-23) or minute (0-59).
      void set_timer_register(uint8_t reg, uint8_t value);
      void set_timer_active(bool active);

    protected:
      bool _writable = true;
      bool stopStatus = false;
      uint32_t last_send_time_ = 0;
      // If a polled command gets no reply within this window we stop waiting and
      // move on, so a register the robot never answers can't stall the cycle.
      static constexpr uint32_t UART_REPLY_TIMEOUT_MS = 2000;

      // Round-robin cursors for the two poll tiers plus the interleave counter.
      size_t fast_index_ = 0;
      size_t slow_index_ = 0;
      uint8_t scheduler_tick_ = 0;
      // One in every SLOW_POLL_EVERY sends is taken from the slow list, so the
      // rarely-changing registers (timers, clock, ...) refresh without diluting
      // the responsive ones (status, voltage, ...).
      static constexpr uint8_t SLOW_POLL_EVERY = 3;

      // Latest raw value per register address, keyed by the decoded address.
      std::map<uint16_t, uint16_t> register_values_;

      template_::TemplateSensor *battery_level_sensor_ = nullptr;
      template_::TemplateSensor *battery_temperature_sensor_ = nullptr;
      template_::TemplateSensor *battery_used_sensor_ = nullptr;
      template_::TemplateSensor *battery_voltage_sensor_ = nullptr;
      template_::TemplateSensor *blade_motor_speed_sensor_ = nullptr;
      template_::TemplateSensor *charging_time_sensor_ = nullptr;
      template_::TemplateSensor *mowing_time_sensor_ = nullptr;
      template_::TemplateSensor *firmware_version_sensor_ = nullptr;

      template_::TemplateTextSensor *mode_text_sensor_ = nullptr;
      template_::TemplateTextSensor *status_text_sensor_ = nullptr;
      template_::TemplateTextSensor *last_code_received_text_sensor_ = nullptr;

      // ---- Write commands (high bit of the address byte set) ----
      static constexpr uint8_t MAN_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x00};
      static constexpr uint8_t AUTO_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x01};
      static constexpr uint8_t HOME_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x03};
      static constexpr uint8_t DEMO_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x04};
      static constexpr uint8_t STOP_ON_DATA[5] = {0x0F, 0x81, 0x2F, 0x00, 0x02};
      static constexpr uint8_t STOP_OFF_DATA[5] = {0x0F, 0x81, 0x2F, 0x00, 0x00};
      static constexpr uint8_t KEY_BACK[5] = {0x0F, 0x80, 0x5F, 0x00, 0x0F};
      static constexpr uint8_t KEY_YES[5] = {0x0F, 0x80, 0x5F, 0x00, 0x12};

      // ---- Read commands: fast tier (frequently changing) ----
      static constexpr uint8_t getStatusCode[5] = {0x0F, 0x01, 0xF1, 0x00, 0x00};
      static constexpr uint8_t getModeCmd[5] = {0x0F, 0x01, 0x2C, 0x00, 0x00};
      static constexpr uint8_t getBatteryVoltage[5] = {0x0F, 0x2E, 0xF4, 0x00, 0x00};
      static constexpr uint8_t getBatteryCurrent[5] = {0x0F, 0x01, 0xEB, 0x00, 0x00};
      static constexpr uint8_t getBladeMotorSpeed[5] = {0x0F, 0x2E, 0xEA, 0x00, 0x00};
      static constexpr uint8_t getBatteryUsed[5] = {0x0F, 0x2E, 0xE0, 0x00, 0x00};
      static constexpr uint8_t READ_STOP_CMD[5] = {0x0F, 0x01, 0x2F, 0x00, 0x00};

      // ---- Read commands: slow tier (rarely changing / bulky) ----
      static constexpr uint8_t getBatteryCharged[5] = {0x0F, 0x01, 0xEF, 0x00, 0x00};
      static constexpr uint8_t getBatteryCapacity[5] = {0x0F, 0x00, 0xB1, 0x00, 0x00};
      static constexpr uint8_t getBatteryReturn[5] = {0x0F, 0x01, 0xF0, 0x00, 0x00};
      static constexpr uint8_t getChargingTime[5] = {0x0F, 0x01, 0xEC, 0x00, 0x00};
      static constexpr uint8_t getChargeCycleMin[5] = {0x0F, 0x02, 0x34, 0x00, 0x00};
      static constexpr uint8_t getMowingTime[5] = {0x0F, 0x00, 0x56, 0x00, 0x00};
      static constexpr uint8_t getBatteryTemperature[5] = {0x0F, 0x02, 0x33, 0x00, 0x00};
      static constexpr uint8_t getBatteryTempLoad[5] = {0x0F, 0x02, 0x35, 0x00, 0x00};
      static constexpr uint8_t getTempNextCheck[5] = {0x0F, 0x02, 0x36, 0x00, 0x00};
      static constexpr uint8_t getCuttingTime[5] = {0x0F, 0x00, 0x38, 0x00, 0x00};
      static constexpr uint8_t getLoopQuality[5] = {0x0F, 0x01, 0xB9, 0x00, 0x00};
      static constexpr uint8_t getSquareStatus[5] = {0x0F, 0x01, 0x38, 0x00, 0x00};
      static constexpr uint8_t getSquarePercent[5] = {0x0F, 0x01, 0x34, 0x00, 0x00};
      static constexpr uint8_t getSquareRef[5] = {0x0F, 0x01, 0x37, 0x00, 0x00};
      static constexpr uint8_t getSpeedRight[5] = {0x0F, 0x24, 0xBF, 0x00, 0x00};
      static constexpr uint8_t getSpeedLeft[5] = {0x0F, 0x24, 0xC0, 0x00, 0x00};
      static constexpr uint8_t getFirmwareVersion[5] = {0x0F, 0x33, 0x90, 0x00, 0x00};
      static constexpr uint8_t getLanguage[5] = {0x0F, 0x3A, 0xC0, 0x00, 0x00};
      static constexpr uint8_t getClockSec[5] = {0x0F, 0x36, 0xB1, 0x00, 0x00};
      static constexpr uint8_t getClockMin[5] = {0x0F, 0x36, 0xB3, 0x00, 0x00};
      static constexpr uint8_t getClockHour[5] = {0x0F, 0x36, 0xB5, 0x00, 0x00};
      static constexpr uint8_t getClockDay[5] = {0x0F, 0x36, 0xB7, 0x00, 0x00};
      static constexpr uint8_t getClockMonth[5] = {0x0F, 0x36, 0xB9, 0x00, 0x00};
      static constexpr uint8_t getClockYear[5] = {0x0F, 0x36, 0xBD, 0x00, 0x00};
      static constexpr uint8_t getTimerActive[5] = {0x0F, 0x4A, 0x4E, 0x00, 0x00};
      static constexpr uint8_t getTimerDays[5] = {0x0F, 0x4A, 0x50, 0x00, 0x00};

      // 16 timer registers 0x4A38..0x4A47 (t1/t2 x weekday/weekend x start/stop x h/m).
      static constexpr uint8_t TIMER_READ_CMDS[16][5] = {
          {0x0F, 0x4A, 0x38, 0x00, 0x00}, {0x0F, 0x4A, 0x39, 0x00, 0x00},
          {0x0F, 0x4A, 0x3A, 0x00, 0x00}, {0x0F, 0x4A, 0x3B, 0x00, 0x00},
          {0x0F, 0x4A, 0x3C, 0x00, 0x00}, {0x0F, 0x4A, 0x3D, 0x00, 0x00},
          {0x0F, 0x4A, 0x3E, 0x00, 0x00}, {0x0F, 0x4A, 0x3F, 0x00, 0x00},
          {0x0F, 0x4A, 0x40, 0x00, 0x00}, {0x0F, 0x4A, 0x41, 0x00, 0x00},
          {0x0F, 0x4A, 0x42, 0x00, 0x00}, {0x0F, 0x4A, 0x43, 0x00, 0x00},
          {0x0F, 0x4A, 0x44, 0x00, 0x00}, {0x0F, 0x4A, 0x45, 0x00, 0x00},
          {0x0F, 0x4A, 0x46, 0x00, 0x00}, {0x0F, 0x4A, 0x47, 0x00, 0x00}};

      std::vector<const uint8_t *> fastCommands;
      std::vector<const uint8_t *> slowCommands;

      void buildCommandLists();
      void sendNextCommand();
      void checkUartRead();
      void setStopStatusFromCode(uint16_t val);
      void publishMode(uint16_t val);
      void publishStatus(uint16_t val);
      std::string formatHex(uint16_t v);
    };

  } // namespace mower
} // namespace esphome
