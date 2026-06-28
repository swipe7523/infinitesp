#include "infinitesp_text_sensor.h"
#include <cctype>

namespace esphome {
namespace infinitesp {

void InfinitESPTextSensor::on_register_update(uint8_t device_addr, uint16_t register_key) {
  // Hold state display: "until HH:MM PM", "Permanent", or "Schedule"
  if (sensor_type_ == "hold_state") {
    if (register_key != REG_SAM_ZONES)
      return;
    auto *data = parent_->get_register(parent_->get_sam_address(), REG_SAM_ZONES);
    if (!data || data->size() < REG3B03_HOLD_DURATIONS + zone_ * 2)
      return;

    uint8_t idx = zone_ - 1;
    if (!(data->at(REG3B03_ACTIVE_ZONES) & (1 << idx)))
      return;

    uint16_t hold_dur = parent_->get_zone_hold_duration(zone_);

    if (hold_dur == 0) {
      publish_state("Schedule");
    } else if (hold_dur >= InfinitESPComponent::HOLD_PERMANENT) {
      publish_state("Hold - Permanent");
    } else {
      std::string end = parent_->format_hold_end(hold_dur);
      if (!end.empty())
        publish_state("Hold until " + end);
      else
        publish_state("Hold " + std::to_string(hold_dur) + " min");
    }
    return;
  }

  // Zone name from SAM 3B03 register
  if (sensor_type_ == "zone_name") {
    if (register_key != REG_SAM_ZONES)
      return;

    auto *data = parent_->get_register(parent_->get_sam_address(), REG_SAM_ZONES);
    if (!data || data->size() < REG3B03_SIZE)
      return;

    uint8_t idx = zone_ - 1;
    if (!(data->at(REG3B03_ACTIVE_ZONES) & (1 << idx)))
      return;

    uint16_t name_offset = REG3B03_ZONE_NAMES + (idx * 12);
    std::string name;
    for (int i = 0; i < 12; i++) {
      char c = (char) data->at(name_offset + i);
      if (c == 0)
        break;
      name += c;
    }
    // Trim trailing spaces
    while (!name.empty() && name.back() == ' ') {
      name.pop_back();
    }

    if (!name.empty()) {
      publish_state(name);
    }
    return;
  }

  // Fixed-offset C-string fields from the thermostat's WiFi (4608), cloud (4609)
  // and dealer (460A) registers. Each entry is {type, register, min_size, offset}:
  // publish the NUL-terminated string at `offset` once the register holds at
  // least `min_size` bytes. min_size == offset + 1 (the byte at `offset` must
  // exist); a min_size of 1 / offset 0 means "any non-empty payload".
  struct StringField { const char *type; uint16_t reg; size_t min_size; size_t offset; };
  static const StringField string_fields[] = {
    {"tstat_ssid",         REG_TSTAT_WIFI,    25,  24},   // 4608
    {"tstat_hostname",     REG_TSTAT_WIFI,    140, 139},
    {"tstat_wifi_mac",     REG_TSTAT_WIFI,    5,   4},
    {"tstat_cloud_host",   REG_TSTAT_CLOUD,   1,   0},    // 4609
    {"tstat_proxy_server", REG_TSTAT_CLOUD,   68,  67},
    {"tstat_dealer_name",  REG_TSTAT_DEALER,  1,   0},    // 460A
    {"tstat_dealer_brand", REG_TSTAT_DEALER,  51,  50},
    {"tstat_dealer_url",   REG_TSTAT_DEALER,  71,  70},
  };
  for (const auto &sf : string_fields) {
    if (sensor_type_ != sf.type)
      continue;
    if (register_key != sf.reg)
      return;
    auto *data = parent_->get_register(ADDR_THERMOSTAT, sf.reg);
    if (!data || data->size() < sf.min_size)
      return;
    publish_state(extract_cstr(*data, sf.offset));
    return;
  }

  // Comfort profile summary from 400A
  if (sensor_type_ == "comfort_profile") {
    if (register_key != REG_TSTAT_COMFORT)
      return;
    auto *data = parent_->get_register(ADDR_THERMOSTAT, REG_TSTAT_COMFORT);
    if (!data || data->size() < COMFORT_ACTIVITY_COUNT * COMFORT_ENTRY_SIZE)
      return;

    const char *names[] = {"home", "away", "sleep", "wake", "manual"};
    const char *fan_names[] = {"off", "low", "med", "high"};
    // Show temperatures in both °C and °F for universal readability
    // (HA can't auto-convert text sensor strings)
    std::string result;
    for (uint8_t i = 0; i < COMFORT_ACTIVITY_COUNT; i++) {
      uint8_t base = i * COMFORT_ENTRY_SIZE;
      float ht_c = parent_->comfort_byte_to_celsius((*data)[base + 0]);
      float cl_c = parent_->comfort_byte_to_celsius((*data)[base + 1]);
      float ht_f = ht_c * 9.0f / 5.0f + 32.0f;
      float cl_f = cl_c * 9.0f / 5.0f + 32.0f;
      if (i > 0)
        result += "; ";
      char buf[80];
      snprintf(buf, sizeof(buf), "%s: ht=%.0f\xc2\xb0" "F/%.1f\xc2\xb0" "C cl=%.0f\xc2\xb0" "F/%.1f\xc2\xb0" "C fan=%s",
               names[i],
               ht_f, ht_c, cl_f, cl_c,
               (*data)[base + 2] < 4 ? fan_names[(*data)[base + 2]] : "?");
      result += buf;
    }
    publish_state(result);
    return;
  }

  // Fault history from 4202
  // 10 entries × 7 bytes: code(1), source(1), hour(1), minute(1), days_be16(2), status(1)
  // Days since 2013-01-01 epoch. Status bit 7 = active (0=active, 1=cleared), bits 0-6 = occurrence count.
  if (sensor_type_ == "fault_history") {
    if (register_key != REG_TSTAT_FAULTS)
      return;
    auto *data = parent_->get_register(ADDR_THERMOSTAT, REG_TSTAT_FAULTS);
    if (!data || data->size() < 70)
      return;

    const char *source_names[] = {"?", "?", "UI", "?", "?", "?", "?", "?",
                                  "?", "?", "?", "?", "?", "?", "?", "?",
                                  "?", "?", "?", "?", "?", "?", "?", "?",
                                  "?", "?", "?", "?", "?", "?", "?", "?",
                                  "IDU", "?", "?", "?", "?", "?", "?", "?",
                                  "?", "?", "?", "?", "?", "?", "?", "?",
                                  "?", "?", "ODU"};
    std::string result;
    for (int i = 0; i < 10; i++) {
      uint8_t base = i * 7;
      uint8_t code = (*data)[base + 0];
      uint8_t source = (*data)[base + 1];
      uint8_t hour = (*data)[base + 2];
      uint8_t minute = (*data)[base + 3];
      uint16_t days = ((uint16_t) (*data)[base + 4] << 8) | (*data)[base + 5];
      uint8_t status = (*data)[base + 6];
      bool active = !(status & 0x80);  // bit 7: 0=active, 1=cleared
      uint8_t occurrences = status & 0x7F;

      // Skip empty entries (all zeros)
      if (code == 0 && source == 0 && days == 0)
        continue;

      if (!result.empty())
        result += "\n";

      // Convert days since 2013-01-01 to a date string
      // 2013-01-01 epoch, account for leap years
      uint32_t total_days = days;
      int year = 2013;
      while (total_days >= 365) {
        uint16_t year_days = ((year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 366 : 365);
        if (total_days >= year_days) {
          total_days -= year_days;
          year++;
        } else {
          break;
        }
      }
      static const uint8_t mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      uint8_t month = 0;
      while (month < 12) {
        uint8_t dim = mdays[month];
        if (month == 1 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
          dim = 29;
        if (total_days >= dim) {
          total_days -= dim;
          month++;
        } else {
          break;
        }
      }
      uint8_t day = (uint8_t) total_days + 1;
      month++;  // 1-indexed

      const char *src_name = (source < sizeof(source_names) / sizeof(source_names[0]))
                                 ? source_names[source]
                                 : "?";
      char buf[80];
      snprintf(buf, sizeof(buf), "%s code=%02d src=%s %02d:%02d %04d-%02d-%02d occ=%d%s",
               active ? "ACT" : "CLR", code, src_name, hour, minute, year, month, day,
               occurrences, (i == 0 && active) ? " (latest)" : "");
      result += buf;
    }

    if (result.empty())
      result = "No faults";

    publish_state(result);
    return;
  }

  // Manufacture date derived from 0104 serial number
  // Carrier serial format: first 2 digits = week (01-52), next 2 digits = year (00-99)
  // Requires device_address to be set — each physical device needs its own sensor
  if (sensor_type_ == "manufacture_date") {
    if (register_key != REG_DEVICE_INFO)
      return;
    if (target_device_addr_ != 0 && device_addr != target_device_addr_)
      return;
    auto *data = parent_->get_register(device_addr, REG_DEVICE_INFO);
    if (!data || data->size() < 100)
      return;

    // Serial starts at offset 96, extract first 4 digits
    const uint8_t *serial = data->data() + 96;
    if (!std::isdigit(serial[0]) || !std::isdigit(serial[1]) ||
        !std::isdigit(serial[2]) || !std::isdigit(serial[3]))
      return;

    uint8_t week = (serial[0] - '0') * 10 + (serial[1] - '0');
    uint8_t year_short = (serial[2] - '0') * 10 + (serial[3] - '0');
    if (week < 1 || week > 52)
      return;

    // Carrier used 2-digit years. 00-39 → 2000-2039, 40-99 → 1940-1999
    uint16_t year = (year_short < 40) ? (2000 + year_short) : (1900 + year_short);

    // Week → approximate month (midpoint of week)
    static const uint16_t month_cumulative[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    uint16_t day_of_year = (week - 1) * 7 + 3;  // midpoint of the week
    bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    if (leap && day_of_year > 59) day_of_year++;  // shift past Feb 29
    const char *month_names[] = {"January", "February", "March", "April", "May", "June",
                                 "July", "August", "September", "October", "November", "December"};
    uint8_t month = 0;
    for (uint8_t m = 1; m < 12; m++) {
      if (day_of_year < month_cumulative[m])
        break;
      month = m;
    }

    char buf[24];
    snprintf(buf, sizeof(buf), "%s %04u", month_names[month], year);
    publish_state(buf);
    return;
  }
}

} // namespace infinitesp
} // namespace esphome
