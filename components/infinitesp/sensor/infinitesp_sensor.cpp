#include "infinitesp_sensor.h"
#include <cstring>

namespace esphome {
namespace infinitesp {

// Physically-plausible band for 061f superheat/subcooling deltas (°C).
// Typical readings are 0–11 °C; this allows generous margin (~4–5× the real
// max, plus headroom below zero for transients) while sitting far below the
// ~1e20 garbage that corrupt 061f fields produce. Anything outside is dropped.
static constexpr float ODU_DELTA_MIN_C = -40.0f;
static constexpr float ODU_DELTA_MAX_C = 80.0f;

void InfinitESPSensor::on_register_update(uint8_t device_addr, uint16_t register_key) {
  float value = NAN;

  // SAM state registers (3B02): temperature, humidity, outdoor temp
  if (register_key == REG_SAM_STATE) {
    auto *data = parent_->get_register(parent_->get_sam_address(), REG_SAM_STATE);
    if (!data || data->size() < 21)
      return;

    // 0xFF is the "no sensor / not ready" sentinel — the thermostat reports it
    // for unequipped zones and, transiently, for all sensors during its
    // post-reboot warmup. Publishing it raw yields garbage (255 °F → 123.9 °C,
    // 255 %); leave the sensor at its last value (NaN → no publish) instead.
    if (sensor_type_ == "outdoor_temperature") {
      uint8_t raw = data->at(REG3B02_OUTDOOR_TEMP);
      if (raw != 0xFF)
        value = parent_->bus_temp_to_celsius((float) raw);
    } else if (sensor_type_ == "temperature") {
      uint8_t idx = zone_ - 1;
      if (data->at(REG3B02_ACTIVE_ZONES) & (1 << idx)) {
        uint8_t raw = data->at(REG3B02_TEMPS + idx);
        if (raw != 0xFF)
          value = parent_->bus_temp_to_celsius((float) raw);
      }
    } else if (sensor_type_ == "humidity") {
      uint8_t idx = zone_ - 1;
      if (data->at(REG3B02_ACTIVE_ZONES) & (1 << idx)) {
        uint8_t raw = data->at(REG3B02_HUMIDITY + idx);
        if (raw != 0xFF)
          value = (float) raw;
      }
    }
  }

  // Thermostat vacation settings (4012): min at byte 0, max at byte 1.
  if (register_key == REG_TSTAT_VACATION &&
      (sensor_type_ == "vacation_min_temp" || sensor_type_ == "vacation_max_temp")) {
    auto *data = parent_->get_register(ADDR_THERMOSTAT, REG_TSTAT_VACATION);
    if (data && data->size() >= 2)
      value = parent_->setpoint_to_celsius(data->at(sensor_type_ == "vacation_min_temp" ? 0 : 1));
  }

  // Pass-through diagnostic sensors. Each reads one register and publishes its
  // decoder's value verbatim (NAN → no publish). The offset/scale lives in the
  // matching InfinitESPComponent accessor — the single source of truth for the
  // register layout (see infinitesp.h); this table only maps sensor_type →
  // (register, decoder). Source registers: 0306 blower rpm, 0316 airflow cfm,
  // 0604 compressor rpm, 0608 drive freq (0.1 Hz), 060e stage, 0605 commanded
  // stage, 0304 line voltage / operating mode, 060a outdoor fan rpm, 0303
  // suction/discharge pressure (psig), 0625 inverter power (W). Sensors needing
  // a post-decode unit conversion (ODU temps, superheat, 061f) stay below.
  struct SimpleSensor {
    const char *type;
    uint16_t reg;
    float (*decode)(const std::vector<uint8_t> &);
  };
  static const SimpleSensor simple_sensors[] = {
    {"blower_rpm",             REG_IDU_STATUS,     InfinitESPComponent::idu_blower_rpm_},
    {"airflow_cfm",            REG_IDU_CONFIG,     InfinitESPComponent::idu_airflow_cfm_},
    {"compressor_rpm",         REG_ODU_COMP_SPEED, InfinitESPComponent::odu_compressor_rpm_},
    {"compressor_frequency",   REG_ODU_DEMAND,     InfinitESPComponent::odu_compressor_frequency_},
    {"odu_stage",              REG_ODU_STAGE_INFO, InfinitESPComponent::odu_stage_},
    {"odu_commanded_stage",    REG_ODU_CMD_STAGE,  InfinitESPComponent::odu_commanded_stage_},
    {"odu_line_voltage",       REG_ODU_STATUS3,    InfinitESPComponent::odu_line_voltage_},
    {"odu_operating_mode",     REG_ODU_STATUS3,    InfinitESPComponent::odu_operating_mode_},
    {"odu_fan_rpm",            REG_ODU_FAN,        InfinitESPComponent::odu_outdoor_fan_rpm_},
    {"odu_dc_bus_voltage",     REG_ODU_FAN,        InfinitESPComponent::odu_dc_bus_voltage_},
    {"odu_ac_line_current",    REG_ODU_FAN,        InfinitESPComponent::odu_ac_line_current_},
    {"odu_suction_pressure",   REG_ODU_STATUS2,    InfinitESPComponent::odu_suction_pressure_psig_},
    {"odu_discharge_pressure", REG_ODU_STATUS2,    InfinitESPComponent::odu_discharge_pressure_psig_},
    {"odu_power",              REG_ODU_POWER,      InfinitESPComponent::odu_power_w_},
  };
  for (const auto &s : simple_sensors) {
    if (register_key == s.reg && sensor_type_ == s.type) {
      auto *data = parent_->get_register(device_addr, s.reg);
      if (data) {
        float v = s.decode(*data);
        if (!std::isnan(v))
          value = v;
      }
      break;
    }
  }

  // ODU IEEE754 float32 values from register 061f, always native °F. Convert to °C.
  // Layout via accessor odu_float_(idx): idx 1..6 at offset 1+(idx-1)*4.
  //   1: superheat target  2: superheat actual  3: subcooling target
  //   4: subcooling actual 5: discharge superheat (all °F deltas)
  //   6: dimensionless constant
  if (register_key == REG_ODU_FLOATS && sensor_type_.rfind("odu_float_", 0) == 0) {
    auto *data = parent_->get_register(device_addr, REG_ODU_FLOATS);
    if (data) {
      int idx = sensor_type_[10] - '0';  // odu_float_N → N
      if (idx >= 1 && idx <= 6) {
        float fval = parent_->odu_float_(*data, idx);
        if (!std::isnan(fval)) {
          if (idx <= 5) {
            // 061f floats are °F superheat/subcooling deltas. Some fields
            // (notably idx 3, subcooling target) decode to absurd ~1e20
            // magnitudes in certain compressor states — this register's
            // layout is only partially reverse-engineered (cf. float 6 =
            // "unk"). Drop physically-impossible readings so the HA sensor
            // holds its last sane value instead of publishing garbage. No
            // real refrigerant superheat/subcooling delta lands outside this
            // band; the corruption is orders of magnitude beyond it.
            float celsius = fval * (5.0f / 9.0f);  // °F delta → °C delta (no -32 offset)
            if (celsius >= ODU_DELTA_MIN_C && celsius <= ODU_DELTA_MAX_C)
              value = celsius;
          } else {
            value = fval;  // float 6 is dimensionless
          }
        }
      }
    }
  }

  // ODU register 0302 temperatures: int16 BE / 16, native °F → °C. Field idx via
  // odu_status1_meas_f_(idx): 0=outdoor 1=coil 2=suction 5=discharge are real
  // temps; idx 3/4 are NOT subcooling/indoor-ambient — cross-referenced against
  // Anantha MQTT, those offsets decode to ~329°F/348°F (non-temperature data) and
  // were publishing ~182°C/175°C garbage. odu_status1_temp_f_ band-rejects idx
  // 3/4 → NAN → no publish, until the real offsets are found. (0302 is not a
  // clean 6-slot temp array; only idx 0/1/2/5 are temps.) idx 3 is a ΔT delta.
  struct OduTemp { const char *type; uint8_t idx; bool guarded; bool is_delta; };
  static const OduTemp odu_temps[] = {
    {"odu_outdoor_temp",        0, false, false},
    {"odu_coil_temp",           1, false, false},
    {"odu_suction_temp",        2, false, false},
    {"odu_subcooling_degf_int", 3, true,  true},   // band-guarded, ΔT delta (no -32)
    {"odu_indoor_ambient",      4, true,  false},  // band-guarded
    {"odu_discharge_temp",      5, false, false},
  };
  if (register_key == REG_ODU_STATUS1) {
    auto *data = parent_->get_register(device_addr, REG_ODU_STATUS1);
    if (data) {
      for (const auto &t : odu_temps) {
        if (sensor_type_ != t.type)
          continue;
        float f = t.guarded ? parent_->odu_status1_temp_f_(*data, t.idx)
                            : parent_->odu_status1_meas_f_(*data, t.idx);
        if (!std::isnan(f))
          value = t.is_delta ? f * (5.0f / 9.0f) : (f - 32.0f) * (5.0f / 9.0f);
        break;
      }
    }
  }

  // Live suction superheat from ODU register 0613 data[52] (float32 BE, °F delta).
  // Supersedes the static 061F idx2 target. Convert °F delta → °C delta (no -32).
  if (register_key == REG_ODU_SUPERHEAT && sensor_type_ == "odu_suction_superheat") {
    auto *data = parent_->get_register(device_addr, REG_ODU_SUPERHEAT);
    if (data) {
      float f = parent_->odu_suction_superheat_f_(*data);
      if (!std::isnan(f)) value = f * (5.0f / 9.0f);
    }
  }

  // ODU inverter module temps from register 060A (data[110] PFCM, data[112] IPM;
  // u16 BE /16, native °F → °C). Cross-validated vs Anantha MQTT pfcm_temp/ipm_temp.
  if (register_key == REG_ODU_FAN && sensor_type_ == "odu_ipm_temp") {
    auto *data = parent_->get_register(device_addr, REG_ODU_FAN);
    if (data) {
      float f = parent_->odu_ipm_temp_f_(*data);
      if (!std::isnan(f)) value = (f - 32.0f) * (5.0f / 9.0f);
    }
  }
  if (register_key == REG_ODU_FAN && sensor_type_ == "odu_pfcm_temp") {
    auto *data = parent_->get_register(device_addr, REG_ODU_FAN);
    if (data) {
      float f = parent_->odu_pfcm_temp_f_(*data);
      if (!std::isnan(f)) value = (f - 32.0f) * (5.0f / 9.0f);
    }
  }

  // --- ZC zone temperatures (register 0302, ZC device address 0x60) ---
  // Per-zone: [tag, id, value_hi, value_lo] where °F = uint16_BE / 16
  if (register_key == REG_ZC_ZONE_STATUS && sensor_type_ == "zc_zone_temperature") {
    auto *data = parent_->get_register(device_addr, REG_ZC_ZONE_STATUS);
    if (data && data->size() == 24 && zone_ >= 2 && zone_ <= 4) {
      uint8_t off_hi = 4 + (zone_ - 2) * 4 + 2;
      uint16_t raw = InfinitESPComponent::decode_u16_be_(*data, off_hi);
      float temp_f = (float) raw / ZC_TEMP_SCALE;
      value = (temp_f - 32.0f) * (5.0f / 9.0f);  // °F → °C for HA
    }
  }

  // --- Cycle counters and runtime hours (registers 0310/0311) ---
  // Format: sequence of 4-byte entries: [key, b1, b2, b3]
  // where value = (b1 << 16) | (b2 << 8) | b3 (24-bit unsigned)
  //
  // IDU keys: 0x23=low_heat, 0x24=high_heat, 0x48=med_heat,
  //           0x2B=poweron, 0x2D=blower
  // ODU keys: 0x23=heat, 0x28=cool, 0x3C=defrost, 0x2B=poweron
  // _cycles = register 0310, _hours = register 0311
  if (register_key == REG_IDU_CYCLES || register_key == REG_IDU_RUNTIME ||
      register_key == REG_ODU_CYCLES || register_key == REG_ODU_RUNTIME) {
    struct KVMap { const char *suffix; uint16_t reg; uint8_t key; };
    static const KVMap kv_map[] = {
      // IDU cycles (0310)
      {"idu_low_heat_cycles",  REG_IDU_CYCLES,  0x23},
      {"idu_high_heat_cycles", REG_IDU_CYCLES,  0x24},
      {"idu_med_heat_cycles",  REG_IDU_CYCLES,  0x48},
      {"idu_poweron_cycles",   REG_IDU_CYCLES,  0x2B},
      {"idu_blower_cycles",    REG_IDU_CYCLES,  0x2D},
      // IDU hours (0311)
      {"idu_low_heat_hours",   REG_IDU_RUNTIME, 0x25},
      {"idu_high_heat_hours",  REG_IDU_RUNTIME, 0x26},
      {"idu_med_heat_hours",   REG_IDU_RUNTIME, 0x49},
      {"idu_poweron_hours",    REG_IDU_RUNTIME, 0x2C},
      {"idu_blower_hours",     REG_IDU_RUNTIME, 0x2E},
      // ODU cycles (0310)
      {"odu_heat_cycles",      REG_ODU_CYCLES,  0x23},
      {"odu_cool_cycles",      REG_ODU_CYCLES,  0x28},
      {"odu_defrost_cycles",   REG_ODU_CYCLES,  0x3C},
      {"odu_poweron_cycles",   REG_ODU_CYCLES,  0x2B},
      // ODU hours (0311)
      {"odu_heat_hours",       REG_ODU_RUNTIME, 0x25},
      {"odu_cool_hours",       REG_ODU_RUNTIME, 0x2A},
      {"odu_defrost_hours",    REG_ODU_RUNTIME, 0x3D},
      {"odu_poweron_hours",    REG_ODU_RUNTIME, 0x2C},
    };

    for (const auto &km : kv_map) {
      if (sensor_type_ == km.suffix) {
        // 0310/0311 are shared register numbers: both the IDU and ODU publish
        // cycle/runtime counters under them, and some byte-keys collide (0x23
        // heat, 0x2B poweron). notify_entities_ fans out by register number, so
        // gate on device role to consume only the matching unit's counters.
        bool role_ok = (sensor_type_.rfind("idu_", 0) == 0)
                           ? parent_->is_idu_addr(device_addr)
                           : parent_->is_odu_addr(device_addr);
        if (role_ok) {
          auto *data = parent_->get_register(device_addr, km.reg);
          if (data && data->size() >= 4) {
            for (size_t i = 0; i + 3 < data->size(); i += 4) {
              if ((*data)[i] == km.key) {
                uint32_t val = ((uint32_t)(*data)[i+1] << 16) |
                               ((uint32_t)(*data)[i+2] << 8) |
                               (uint32_t)(*data)[i+3];
                value = (float) val;
                break;
              }
            }
          }
        }
        break;
      }
    }
  }

  if (!std::isnan(value)) {
    publish_state(value);
  }
}

} // namespace infinitesp
} // namespace esphome
