#include "automower.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace esphome
{
    namespace mower
    {

        Automower::Automower(uart::UARTComponent *parent, uint32_t update_interval)
            : PollingComponent(update_interval), uart::UARTDevice(parent)
        {

            battery_temperature_sensor_ = new template_::TemplateSensor();
            battery_level_sensor_ = new template_::TemplateSensor();
            battery_used_sensor_ = new template_::TemplateSensor();
            battery_voltage_sensor_ = new template_::TemplateSensor();
            blade_motor_speed_sensor_ = new template_::TemplateSensor();
            charging_time_sensor_ = new template_::TemplateSensor();
            mowing_time_sensor_ = new template_::TemplateSensor();
            firmware_version_sensor_ = new template_::TemplateSensor();

            last_code_received_text_sensor_ = new template_::TemplateTextSensor();
            mode_text_sensor_ = new template_::TemplateTextSensor();
            status_text_sensor_ = new template_::TemplateTextSensor();

            buildCommandLists();
        }

        void Automower::buildCommandLists()
        {
            fastCommands = {
                getStatusCode,
                getModeCmd,
                getBatteryVoltage,
                getBatteryCurrent,
                getBladeMotorSpeed,
                getBatteryUsed,
                READ_STOP_CMD};

            slowCommands = {
                getBatteryCharged,
                getBatteryCapacity,
                getBatteryReturn,
                getChargingTime,
                getChargeCycleMin,
                getMowingTime,
                getBatteryTemperature,
                getBatteryTempLoad,
                getTempNextCheck,
                getCuttingTime,
                getLoopQuality,
                getSquareStatus,
                getSquarePercent,
                getSquareRef,
                getSpeedRight,
                getSpeedLeft,
                getFirmwareVersion,
                getLanguage,
                getClockSec,
                getClockMin,
                getClockHour,
                getClockDay,
                getClockMonth,
                getClockYear,
                getTimerActive,
                getTimerDays};
            for (int i = 0; i < 8; i++)
                slowCommands.push_back(TIMER_READ_CMDS[i]);
        }

        void Automower::set_battery_temperature_sensor(template_::TemplateSensor *s) { battery_temperature_sensor_ = s; }
        void Automower::set_battery_level_sensor(template_::TemplateSensor *s) { battery_level_sensor_ = s; }
        void Automower::set_battery_used_sensor(template_::TemplateSensor *s) { battery_used_sensor_ = s; }
        void Automower::set_battery_voltage_sensor(template_::TemplateSensor *s) { battery_voltage_sensor_ = s; }
        void Automower::set_blade_motor_speed_sensor(template_::TemplateSensor *s) { blade_motor_speed_sensor_ = s; }
        void Automower::set_charging_time_sensor(template_::TemplateSensor *s) { charging_time_sensor_ = s; }
        void Automower::set_mowing_time_sensor(template_::TemplateSensor *s) { mowing_time_sensor_ = s; }
        void Automower::set_firmware_version_sensor(template_::TemplateSensor *s) { firmware_version_sensor_ = s; }

        void Automower::set_last_code_received_text_sensor(template_::TemplateTextSensor *s) { last_code_received_text_sensor_ = s; }
        void Automower::set_mode_text_sensor(template_::TemplateTextSensor *s) { mode_text_sensor_ = s; }
        void Automower::set_status_text_sensor(template_::TemplateTextSensor *s) { status_text_sensor_ = s; }

        template_::TemplateSensor *Automower::get_battery_temperature_sensor() const { return battery_temperature_sensor_; }
        template_::TemplateSensor *Automower::get_blade_motor_speed_sensor() const { return blade_motor_speed_sensor_; }
        template_::TemplateSensor *Automower::get_battery_level_sensor() const { return battery_level_sensor_; }
        template_::TemplateSensor *Automower::get_battery_used_sensor() const { return battery_used_sensor_; }
        template_::TemplateSensor *Automower::get_battery_voltage_sensor() const { return battery_voltage_sensor_; }
        template_::TemplateSensor *Automower::get_charging_time_sensor() const { return charging_time_sensor_; }
        template_::TemplateSensor *Automower::get_mowing_time_sensor() const { return mowing_time_sensor_; }
        template_::TemplateSensor *Automower::get_firmware_version_sensor() const { return firmware_version_sensor_; }

        template_::TemplateTextSensor *Automower::get_last_code_received_text_sensor() const { return last_code_received_text_sensor_; }
        template_::TemplateTextSensor *Automower::get_mode_text_sensor() const { return mode_text_sensor_; }
        template_::TemplateTextSensor *Automower::get_status_text_sensor() const { return status_text_sensor_; }

        void Automower::store_register(uint16_t addr, uint16_t val)
        {
            auto it = std::lower_bound(register_values_.begin(), register_values_.end(), addr,
                                       [](const std::pair<uint16_t, uint16_t> &e, uint16_t a)
                                       { return e.first < a; });
            if (it != register_values_.end() && it->first == addr)
                it->second = val;
            else
                register_values_.insert(it, std::make_pair(addr, val));
        }

        float Automower::get_register(uint16_t addr, bool is_signed)
        {
            auto it = std::lower_bound(register_values_.begin(), register_values_.end(), addr,
                                       [](const std::pair<uint16_t, uint16_t> &e, uint16_t a)
                                       { return e.first < a; });
            if (it == register_values_.end() || it->first != addr)
                return NAN;
            uint16_t raw = it->second;
            if (is_signed)
                return static_cast<float>(static_cast<int16_t>(raw));
            return static_cast<float>(raw);
        }

        esphome::optional<esphome::ESPTime> Automower::get_timer_time(uint8_t reg)
        {
            float v = get_register(0x4A00 | reg);
            if (std::isnan(v))
                return {};
            // One timer time is one 16-bit register at an even address: hour in
            // the low byte, minute in the high byte (09:15 reads back 0x0F09).
            // Verified against the keypad display on a 220AC.
            uint16_t raw = static_cast<uint16_t>(v);
            float h = static_cast<float>(raw & 0xFF);
            float m = static_cast<float>(raw >> 8);
            // A register that has not been written yet, or a garbled reply, can
            // hold anything. Report unknown rather than a nonsense clock time.
            if (h < 0.0f || h > 23.0f || m < 0.0f || m > 59.0f)
                return {};
            esphome::ESPTime t{};
            t.hour = static_cast<uint8_t>(h);
            t.minute = static_cast<uint8_t>(m);
            t.second = 0;
            return t;
        }

        esphome::optional<bool> Automower::get_timer_bypass()
        {
            float v = get_register(0x4A4E);
            if (std::isnan(v))
                return {};
            // The keypad's "Bypass timer" box: 1 when ticked (the schedule is
            // ignored), 0 when the schedule is in use.
            return v != 0.0f;
        }

        esphome::optional<bool> Automower::get_week_day(uint8_t bit)
        {
            float v = get_register(0x4A50);
            if (std::isnan(v) || bit > 6)
                return {};
            return ((static_cast<uint16_t>(v) >> bit) & 0x01) != 0;
        }

        // ---- Register scanner ----

        void Automower::reset_scan()
        {
            scan_addr_ = scan_start_;
            scan_pass_ = 0;
            scan_changes_ = 0;
            scan_no_reply_ = 0;
            scan_pending_ = false;
            scan_values_.clear();
            scan_recent_.clear();
        }

        void Automower::set_scan_enabled(bool enabled)
        {
            if (enabled && !scan_enabled_)
            {
                reset_scan();
                ESP_LOGI("Automower", "SCAN start 0x%04X-0x%04X", scan_start_, scan_end_);
            }
            else if (!enabled && scan_enabled_)
            {
                ESP_LOGI("Automower", "SCAN stop after %u passes, %u changes", (unsigned) scan_pass_, (unsigned) scan_changes_);
            }
            scan_enabled_ = enabled;
        }

        bool Automower::set_scan_range(const std::string &range)
        {
            unsigned start = 0, end = 0;
            if (sscanf(range.c_str(), "%x-%x", &start, &end) != 2 || start > end || end > 0x7FFF ||
                end - start >= SCAN_MAX_SPAN)
            {
                ESP_LOGW("Automower", "SCAN range '%s' rejected: want hex START-END, END <= 7FFF, at most %u addresses",
                         range.c_str(), (unsigned) SCAN_MAX_SPAN);
                return false;
            }
            scan_start_ = static_cast<uint16_t>(start);
            scan_end_ = static_cast<uint16_t>(end);
            reset_scan();
            ESP_LOGI("Automower", "SCAN range 0x%04X-0x%04X", scan_start_, scan_end_);
            return true;
        }

        std::string Automower::get_scan_range() const
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%04X-%04X", scan_start_, scan_end_);
            return std::string(buf);
        }

        std::string Automower::get_scan_status() const
        {
            if (!scan_enabled_)
                return "off";
            char buf[96];
            snprintf(buf, sizeof(buf), "pass %u at %04X, %u read, %u silent, %u changes",
                     (unsigned) scan_pass_ + 1, scan_addr_, (unsigned) scan_values_.size(),
                     (unsigned) scan_no_reply_, (unsigned) scan_changes_);
            std::string s(buf);
            // Newest changes first, kept short enough for a HA state (255).
            for (auto it = scan_recent_.rbegin(); it != scan_recent_.rend(); ++it)
            {
                if (s.size() + it->size() + 2 > 250)
                    break;
                s += "; " + *it;
            }
            return s;
        }

        void Automower::sendScanFrame()
        {
            // Skip the keypad's own addresses; reads only, the write bit stays clear.
            while (scan_addr_ == 0x005F || scan_addr_ == 0x0F80)
                advanceScan();
            uint8_t data[5] = {0x0F, static_cast<uint8_t>((scan_addr_ >> 8) & 0x7F),
                               static_cast<uint8_t>(scan_addr_ & 0xFF), 0x00, 0x00};
            scan_pending_ = true;
            sendFrame(data, true, SCAN_REPLY_TIMEOUT_MS);
        }

        void Automower::onScanReply(uint16_t addr, uint16_t val)
        {
            auto it = std::lower_bound(scan_values_.begin(), scan_values_.end(), addr,
                                       [](const std::pair<uint16_t, uint16_t> &e, uint16_t a)
                                       { return e.first < a; });
            if (it != scan_values_.end() && it->first == addr)
            {
                if (it->second != val)
                {
                    ESP_LOGW("Automower", "SCAN CHANGE 0x%04X: 0x%04X -> 0x%04X", addr, it->second, val);
                    char buf[24];
                    snprintf(buf, sizeof(buf), "%04X:%04X>%04X", addr, it->second, val);
                    scan_recent_.push_back(buf);
                    if (scan_recent_.size() > 12)
                        scan_recent_.erase(scan_recent_.begin());
                    scan_changes_++;
                    it->second = val;
                }
            }
            else
            {
                scan_values_.insert(it, std::make_pair(addr, val));
                ESP_LOGI("Automower", "SCAN 0x%04X = 0x%04X", addr, val);
            }
            advanceScan();
        }

        void Automower::advanceScan()
        {
            if (scan_addr_ >= scan_end_)
            {
                ESP_LOGI("Automower", "SCAN pass %u done: %u read, %u silent, %u changes", (unsigned) scan_pass_ + 1,
                         (unsigned) scan_values_.size(), (unsigned) scan_no_reply_, (unsigned) scan_changes_);
                scan_addr_ = scan_start_;
                scan_pass_++;
                scan_no_reply_ = 0;
            }
            else
            {
                scan_addr_++;
            }
        }

        void Automower::setup() {}

        void Automower::dump_config()
        {
            ESP_LOGCONFIG("Automower", "Automower:");
            ESP_LOGCONFIG("Automower", "  Update interval: %u ms", (unsigned) this->get_update_interval());
            ESP_LOGCONFIG("Automower", "  Fast registers: %u", (unsigned) fastCommands.size());
            ESP_LOGCONFIG("Automower", "  Slow registers: %u", (unsigned) slowCommands.size());
            ESP_LOGCONFIG("Automower", "  Slow poll every: %u sends", (unsigned) SLOW_POLL_EVERY);
            ESP_LOGCONFIG("Automower", "  Register scan: %s, range %s", scan_enabled_ ? "on" : "off",
                          get_scan_range().c_str());
        }

        // Sending is driven from loop(), not from this tick. update_interval is
        // the minimum gap between two frames rather than a fixed schedule, so
        // the pace follows how fast the machine actually answers while still
        // never exceeding a rate it can keep up with.
        void Automower::update() {}

        void Automower::serviceBus()
        {
            const uint32_t now = millis();

            // The service port is half duplex and the robot's own keypad talks
            // on it too, so only one frame may be in flight. Wait for the answer
            // before sending anything else, and give up after the timeout so a
            // register the robot never answers cannot stall the cycle.
            if (awaiting_reply_)
            {
                if ((now - last_send_time_) < reply_timeout_ms_)
                    return;
                awaiting_reply_ = false;
                if (scan_pending_)
                {
                    scan_pending_ = false;
                    scan_no_reply_++;
                    ESP_LOGD("Automower", "SCAN 0x%04X no reply", expected_addr_);
                    advanceScan();
                }
                else
                {
                    ESP_LOGD("Automower", "No reply for 0x%04X, moving on", expected_addr_);
                }
            }

            if ((now - last_send_time_) < this->get_update_interval())
                return;

            // User writes go first: a button press should not wait out a full
            // poll cycle.
            if (!write_queue_.empty())
            {
                Frame f = write_queue_.front();
                write_queue_.erase(write_queue_.begin());
                sendFrame(f.b, false);
                return;
            }

            sendNextPoll();
        }

        void Automower::sendNextPoll()
        {
            // While scanning, two of every three sends go to the scan so a pass
            // finishes in reasonable time; the normal poll keeps the rest.
            if (scan_enabled_)
            {
                if (++scan_tick_ >= 3)
                    scan_tick_ = 0;
                if (scan_tick_ != 0)
                {
                    sendScanFrame();
                    return;
                }
            }
            const uint8_t *cmd;
            // Interleave one slow-tier command every SLOW_POLL_EVERY sends.
            if (++scheduler_tick_ >= SLOW_POLL_EVERY && !slowCommands.empty())
            {
                scheduler_tick_ = 0;
                cmd = slowCommands[slow_index_];
                slow_index_ = (slow_index_ + 1) % slowCommands.size();
            }
            else
            {
                if (fastCommands.empty())
                    return;
                cmd = fastCommands[fast_index_];
                fast_index_ = (fast_index_ + 1) % fastCommands.size();
            }
            sendFrame(cmd, true);
        }

        void Automower::sendFrame(const uint8_t *data, bool expect_reply, uint32_t timeout_ms)
        {
            ESP_LOGD("Automower", "UART TX: %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4]);
            write_array(data, 5);
            last_send_time_ = millis();
            awaiting_reply_ = expect_reply;
            reply_timeout_ms_ = timeout_ms;
            if (expect_reply)
                expected_addr_ = ((data[1] & 0x7F) << 8) | data[2];
        }

        void Automower::queueFrame(const uint8_t *data)
        {
            Frame f{};
            for (uint8_t i = 0; i < 5; i++)
                f.b[i] = data[i];
            write_queue_.push_back(f);
        }

        void Automower::loop()
        {
            checkUartRead();
            serviceBus();
        }

        void Automower::set_mode(const std::string &value)
        {
            if (value == "MAN")
            {
                queueFrame(MAN_DATA);
            }
            else if (value == "AUTO")
            {
                queueFrame(AUTO_DATA);
            }
            else if (value == "HOME")
            {
                queueFrame(HOME_DATA);
            }
            else if (value == "DEMO")
            {
                queueFrame(DEMO_DATA);
            }
            else
            {
                ESP_LOGE("Automower", "Unknown mode: %s", value.c_str());
            }
        }

        void Automower::set_stop(bool stop)
        {
            queueFrame(stop ? STOP_ON_DATA : STOP_OFF_DATA);
        }

        void Automower::set_left_motor(int value)
        {
            uint8_t data[5] = {0x0F, 0x92, 0x23, static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF)};
            queueFrame(data);
        }

        void Automower::set_right_motor(int value)
        {
            uint8_t data[5] = {0x0F, 0x92, 0x03, static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF)};
            queueFrame(data);
        }

        void Automower::key_back() { queueFrame(KEY_BACK); }
        void Automower::key_yes() { queueFrame(KEY_YES); }
        void Automower::key_num(uint8_t num)
        {
            uint8_t data[5] = {0x0F, 0x80, 0x5F, 0x00, num};
            queueFrame(data);
        }

        void Automower::set_timer_time(uint8_t reg, uint8_t hour, uint8_t minute)
        {
            // Write address = read address with the high bit set (0x4A -> 0xCA).
            // A write frame carries the value high byte first (mode AUTO is
            // written as 00 01 and reads back as 0x0001), so minute then hour.
            uint8_t data[5] = {0x0F, 0xCA, reg, minute, hour};
            ESP_LOGD("Automower", "Set timer reg 0x%02X = %02u:%02u", reg, hour, minute);
            queueFrame(data);
            // Update the cache too. The entities read their value from it every
            // 60s, while the slow poller only refreshes a timer register every
            // few minutes, so without this a new time reverts on screen until
            // the next read comes round.
            store_register(0x4A00 | reg, static_cast<uint16_t>((minute << 8) | hour));
        }

        void Automower::set_timer_bypass(bool bypass)
        {
            // 1 ticks the keypad's "Bypass timer" box, 0 uses the schedule.
            uint8_t data[5] = {0x0F, 0xCA, 0x4E, 0x00, static_cast<uint8_t>(bypass ? 0x01 : 0x00)};
            ESP_LOGD("Automower", "Set timer bypass = %s", bypass ? "true" : "false");
            queueFrame(data);
            store_register(0x4A4E, bypass ? 0x01 : 0x00);
        }

        void Automower::set_week_day(uint8_t bit, bool on)
        {
            float v = get_register(0x4A50);
            if (std::isnan(v) || bit > 6)
            {
                // Without the current mask a write would clear the other days.
                ESP_LOGW("Automower", "Week timer days not read yet, ignoring change");
                return;
            }
            uint8_t mask = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0x7F);
            if (on)
                mask |= (1 << bit);
            else
                mask &= ~(1 << bit);
            uint8_t data[5] = {0x0F, 0xCA, 0x50, 0x00, mask};
            ESP_LOGD("Automower", "Set week timer days = 0x%02X", mask);
            queueFrame(data);
            store_register(0x4A50, mask);
        }

        void Automower::set_clock(const esphome::ESPTime &t)
        {
            if (!t.is_valid())
            {
                ESP_LOGW("Automower", "No valid time to set the mower clock from");
                return;
            }
            // Same registers as the reads, write bit set (0x36 -> 0xB6); each is
            // a small value in the low byte. Queued writes go out one per poll
            // gap, so the clock lands a few seconds behind; seconds go last.
            const uint8_t regs[6] = {0xBD, 0xB9, 0xB7, 0xB5, 0xB3, 0xB1};
            const uint8_t vals[6] = {static_cast<uint8_t>(t.year % 100), t.month, t.day_of_month,
                                     t.hour, t.minute, t.second};
            for (int i = 0; i < 6; i++)
            {
                uint8_t data[5] = {0x0F, 0xB6, regs[i], 0x00, vals[i]};
                queueFrame(data);
                store_register(0x3600 | regs[i], vals[i]);
            }
            ESP_LOGI("Automower", "Set mower clock to 20%02u-%02u-%02u %02u:%02u:%02u", vals[0], vals[1], vals[2],
                     vals[3], vals[4], vals[5]);
        }

        void Automower::checkUartRead()
        {
            while (available() > 0 && peek() != 0x0F)
                read();
            while (available() >= 5 && peek() == 0x0F)
            {
                uint8_t readData[5];
                read_array(readData, 5);

                ESP_LOGD("Automower", "UART RX: %02X %02X %02X %02X %02X", readData[0], readData[1], readData[2], readData[3], readData[4]);

                uint16_t addr = ((readData[1] & 0x7F) << 8) | readData[2];
                uint16_t val = (readData[4] << 8) | readData[3];

                ESP_LOGD("Automower", "Decoded: addr=0x%04X val=0x%04X", addr, val);

                // Only the answer we asked for releases the bus. The keypad
                // talks on the same wires, and treating its traffic as our
                // reply would let us send while the real answer is still in
                // flight. Anything unmatched falls through to the timeout.
                bool scanned = false;
                if (awaiting_reply_ && addr == expected_addr_)
                {
                    awaiting_reply_ = false;
                    if (scan_pending_)
                    {
                        scan_pending_ = false;
                        scanned = true;
                        onScanReply(addr, val);
                    }
                }

                // Raw address bytes keep the write bit, so an acknowledged
                // write (812C) is distinguishable from a read reply (012C).
                if (last_code_received_text_sensor_ != nullptr)
                {
                    char code[16];
                    snprintf(code, sizeof(code), "%02X%02X=%04X", readData[1], readData[2], val);
                    last_code_received_text_sensor_->publish_state(code);
                }

                // Cache every register (except the keypad's own chatter) so the
                // template sensors/numbers/time entities can read the latest
                // value without polling it themselves.
                if (addr != 0x005F && addr != 0x0F80)
                    store_register(addr, val);

                switch (addr)
                {
                case 0x012C:
                    publishMode(val);
                    break;
                case 0x01F1:
                    publishStatus(val);
                    break;
                case 0x01EC:
                    if (charging_time_sensor_)
                        charging_time_sensor_->publish_state(val);
                    break;
                case 0x0056:
                    if (mowing_time_sensor_)
                        mowing_time_sensor_->publish_state(val);
                    break;
                case 0x01EF:
                    if (battery_level_sensor_)
                        battery_level_sensor_->publish_state(val);
                    break;
                case 0x0233:
                    if (battery_temperature_sensor_)
                        battery_temperature_sensor_->publish_state(val);
                    break;
                case 0x2EE0:
                    if (battery_used_sensor_)
                        battery_used_sensor_->publish_state(val);
                    break;
                case 0x2EEA:
                    if (blade_motor_speed_sensor_)
                        blade_motor_speed_sensor_->publish_state(val);
                    break;
                case 0x2EF4:
                    if (battery_voltage_sensor_)
                        battery_voltage_sensor_->publish_state(val / 1000.0f);
                    break;
                case 0x3390:
                    if (firmware_version_sensor_)
                        firmware_version_sensor_->publish_state(val);
                    break;
                case 0x012F:
                    setStopStatusFromCode(val);
                    break;
                // Raw registers cached above and exposed via get_register() /
                // get_timer_time() / get_timer_bypass() / get_week_day() to the yaml entities.
                case 0x00B1: case 0x01EB: case 0x01F0:
                case 0x0234: case 0x0235: case 0x0236:
                case 0x0038: case 0x01B9:
                case 0x0138: case 0x0134: case 0x0137:
                case 0x24BF: case 0x24C0: case 0x3AC0:
                case 0x36B1: case 0x36B3: case 0x36B5:
                case 0x36B7: case 0x36B9: case 0x36BD:
                case 0x4A4E: case 0x4A50:
                case 0x4A38: case 0x4A3A: case 0x4A3C: case 0x4A3E:
                case 0x4A40: case 0x4A42: case 0x4A44: case 0x4A46:
                    break;
                default:
                    // Scanned addresses are logged by the scanner itself.
                    if (!scanned)
                        ESP_LOGW("Automower", "Unhandled address: 0x%04X with value 0x%04X", addr, val);
                    break;
                }
            }
        }

        void Automower::setStopStatusFromCode(uint16_t val)
        {
            stopStatus = (val == 0x0002);
        }

        void Automower::publishMode(uint16_t val)
        {
            if (!mode_text_sensor_)
                return;
            std::string mode;
            switch (val)
            {
            case 0x0000:
                mode = "MAN";
                break;
            case 0x0001:
                mode = "AUTO";
                break;
            case 0x0002:
                mode = "Charging then AUTO";
                break;
            case 0x0003:
                mode = "HOME";
                break;
            case 0x0004:
                mode = "DEMO";
                break;
            default:
                mode = "MODE_" + formatHex(val);
                break;
            }
            mode_text_sensor_->publish_state(mode);
        }

        void Automower::publishStatus(uint16_t val)
        {
            if (!status_text_sensor_)
                return;
            if (val == 0x0410) // Collission or dodge
                return;
            std::string s;
            switch (val)
            {
            case 0x0010:
                s = "Outside working area";
                break;
            case 0x0012:
                s = "LBV Low battery voltage";
                break;
            case 0x001e:
                s = "Cutting system blocked";
                break;
            case 0x03EA:
                s = "MIP Mowing in progress";
                break;
            case 0x0006:
                s = "Left wheel motor blocked";
                break;
            case 0x0008:
                s = "Right wheel motor blocked";
                break;
            case 0x000C:
                s = "No loop signal";
                break;
            case 0x001A:
                s = "Station blocked";
                break;
            case 0x0022:
                s = "Mower lifted";
                break;
            case 0x0034:
                s = "Station no contact";
                break;
            case 0x0036:
                s = "Pin expired";
                break;
            case 0x03E8:
                s = "Leaving station";
                break;
            case 0x03EE:
                s = "Start mowing";
                break;
            case 0x03F0:
                s = "Mowing started";
                break;
            case 0x03F4:
                s = "Start mowing2";
                break;
            case 0x03F6:
                s = "Charging";
                break;
            case 0x03F8:
                s = "Waiting timer2";
                break;
            case 0x1016:
                s = "Waiting timer";
                break;
            case 0x0400:
                s = "Parking in station";
                break;
            case 0x040C:
                s = "Square mode";
                break;
            case 0x040E:
                s = "Stuck";
                break;
            case 0x0412:
                s = "Searching";
                break;
            case 0x0414:
                s = "Stop";
                break;
            case 0x0418:
                s = "Docking";
                break;
            case 0x041A:
                s = "Leaving station";
                break;
            case 0x041C:
                s = "Error";
                break;
            case 0x0420:
                s = "Waiting for use";
                break;
            case 0x0422:
                s = "Follow boundary";
                break;
            case 0x0424:
                s = "Found N-Signal";
                break;
            case 0x0426:
                s = "Stuck";
                break;
            case 0x0428:
                s = "Searching";
                break;
            case 0x042E:
                s = "Follow guide line";
                break;
            case 0x0430:
                s = "Follow loop wire";
                break;
            default:
                s = "STATUS_" + formatHex(val);
                break;
            }
            status_text_sensor_->publish_state(s);
        }

        std::string Automower::formatHex(uint16_t v)
        {
            char s[16];
            snprintf(s, sizeof(s), "%04x", v);
            return std::string(s);
        }

    } // namespace mower
} // namespace esphome
