#include "infinitesp_climate.h"

namespace esphome {
namespace infinitesp {

climate::ClimateTraits InfinitESPClimate::traits() {
  auto traits = climate::ClimateTraits();
  using namespace esphome::climate;
  traits.add_feature_flags(CLIMATE_SUPPORTS_CURRENT_TEMPERATURE |
                           CLIMATE_SUPPORTS_ACTION |
                           CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE);

  // The ABCD bus works in whole °F. HA treats all ESPHome climate as °C internally.
  // min/max are in °C (HA converts to °F for display via show_temp()).
  //
  // The step is special: HA's ESPHome integration does round(step, 1) on the raw
  // °C value, and the climate entity does NOT convert it to the display unit.
  // So 5/9 (0.5556) becomes 0.6 in the HA UI, not 1.0°F.
  // Fix: set step = 1.0. HA shows 1.0 as the step. When the user clicks +/-, HA
  // adds 1.0 to the °F display value, converts to °C, and sends it to us. Our
  // control() handler rounds to nearest °F regardless.
  static const float STEP_C = 5.0f / 9.0f;  // exactly 1°F
  traits.set_visual_min_temperature((40.0f - 32.0f) * STEP_C);   // 40°F = 4.444°C
  traits.set_visual_max_temperature((99.0f - 32.0f) * STEP_C);   // 99°F = 37.222°C
  traits.set_visual_temperature_step(1.0f);

  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);

  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);

  // Standard presets map to comfort profile activities from the zone's comfort
  // row (thermostat table 40, register 400A+zone-1).
  // NONE = cancel hold (resume schedule).
  // HOME/AWAY/SLEEP/WAKE = apply that activity's setpoints+fan with a permanent hold.
  // The thermostat stores 5 activities: home, away, sleep, wake, manual.
  traits.add_supported_preset(climate::CLIMATE_PRESET_HOME);
  traits.add_supported_preset(climate::CLIMATE_PRESET_AWAY);
  traits.add_supported_preset(climate::CLIMATE_PRESET_SLEEP);

  // Custom presets (including PRESET_VACATION) are registered on the entity (in
  // the constructor) rather than on ClimateTraits — the traits setter was
  // deprecated in 2026.5.0 (removed in 2026.11.0). Climate::get_traits() merges
  // the entity-owned list into the returned traits.

  return traits;
}

void InfinitESPClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    auto mode = call.get_mode().value();
    this->mode = mode;
    uint8_t sys = SYSMODE_OFF;
    switch (mode) {
      case climate::CLIMATE_MODE_HEAT:      sys = SYSMODE_HEAT; break;
      case climate::CLIMATE_MODE_COOL:      sys = SYSMODE_COOL; break;
      case climate::CLIMATE_MODE_HEAT_COOL: sys = SYSMODE_AUTO; break;
      case climate::CLIMATE_MODE_OFF:       sys = SYSMODE_OFF; break;
      default: break;
    }
    sys_mode_ = sys;                // set before set_system_mode so the
                                    // broadcast's self-call is idempotent
    parent_->set_system_mode(sys);  // propagates to all sibling zones
    // Hold this selection until the thermostat confirms it (a poll whose mode
    // nibble matches) or the window expires. Without this, the next bus poll —
    // or the parallel "System Mode" select writing the same register — can
    // revert the mode the instant the user sets it (HA snapback / oscillation).
    pending_mode_ = sys;
    pending_mode_active_ = true;
    pending_mode_until_ms_ = millis() + PENDING_MODE_WINDOW_MS;
  }

  // Setpoint changes arrive ONLY as target_temperature_low/high. This entity
  // declares CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE, and ESPHome's
  // ClimateCall::validate_() unconditionally resets target_temperature_ for a
  // two-point entity ("Cannot set target temperature for climate device with
  // two-point target temperature"), so a single-target branch here would be
  // dead code. HA drives both sliders from low/high regardless of mode.
  if (call.get_target_temperature_low().has_value()) {
    float target_c = call.get_target_temperature_low().value();
    uint8_t target_bus = parent_->celsius_to_setpoint(target_c);
    heat_sp_ = target_bus;
    parent_->set_zone_setpoint(zone_, heat_sp_, cool_sp_);
    this->target_temperature_low = target_c;
    set_pending_setpoint_(heat_sp_, cool_sp_);
  }

  if (call.get_target_temperature_high().has_value()) {
    float target_c = call.get_target_temperature_high().value();
    uint8_t target_bus = parent_->celsius_to_setpoint(target_c);
    cool_sp_ = target_bus;
    parent_->set_zone_setpoint(zone_, heat_sp_, cool_sp_);
    this->target_temperature_high = target_c;
    set_pending_setpoint_(heat_sp_, cool_sp_);
  }

  if (call.get_fan_mode().has_value()) {
    auto fan = call.get_fan_mode().value();
    this->fan_mode = fan;
    uint8_t fm = FAN_AUTO;
    switch (fan) {
      case climate::CLIMATE_FAN_AUTO:   fm = FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW:    fm = FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: fm = FAN_MED; break;
      case climate::CLIMATE_FAN_HIGH:   fm = FAN_HIGH; break;
      default: break;
    }
    parent_->set_zone_fan(zone_, fm);
    fan_mode_ = fm;
  }

  // Handle standard presets — activity-based holds using the zone's comfort row
  // (comfort profiles from 400A). Each maps to a comfort activity applied as a
  // permanent hold.
  if (call.get_preset().has_value()) {
    auto preset = call.get_preset().value();
    struct PresetMap { climate::ClimatePreset preset; uint8_t activity; };
    static const PresetMap preset_map[] = {
      {climate::CLIMATE_PRESET_HOME,  COMFORT_HOME},
      {climate::CLIMATE_PRESET_AWAY,  COMFORT_AWAY},
      {climate::CLIMATE_PRESET_SLEEP, COMFORT_SLEEP},
    };
    for (const auto &pm : preset_map) {
      if (preset != pm.preset)
        continue;
      parent_->apply_activity(zone_, pm.activity, InfinitESPComponent::HOLD_PERMANENT);
      this->set_preset_(preset);
      hold_duration_ = InfinitESPComponent::HOLD_PERMANENT;
      last_activity_ = pm.activity;
      break;
    }
  }

  // Handle custom presets
  if (call.has_custom_preset()) {
    auto custom = call.get_custom_preset();
    if (custom == PRESET_SCHEDULE) {
      // Cancel hold — resume schedule
      parent_->set_zone_hold(zone_, 0);
      hold_duration_ = 0;
      last_activity_ = NO_ACTIVITY;
      this->set_custom_preset_(PRESET_SCHEDULE);
      ESP_LOGI("InfinitESP", "Zone %d: preset PER SCHEDULE → cancel hold", zone_);
    } else if (custom == PRESET_WAKE) {
      parent_->apply_activity(zone_, COMFORT_WAKE, InfinitESPComponent::HOLD_PERMANENT);
      hold_duration_ = InfinitESPComponent::HOLD_PERMANENT;
      last_activity_ = COMFORT_WAKE;
      this->set_custom_preset_(PRESET_WAKE);
      ESP_LOGI("InfinitESP", "Zone %d: preset WAKE → permanent hold", zone_);
    } else if (custom == PRESET_VACATION) {
      // Vacation is reported FROM the bus (setpoint-override detection below);
      // setting it from HA isn't supported yet (would require writing the vacation
      // config and triggering the system-wide override). No-op — the detected
      // state reasserts on the next bus poll.
      ESP_LOGW("InfinitESP", "Zone %d: setting Vacation from HA is not yet supported", zone_);
    }
    // Hold Timer and Hold Indefinitely are read-only states set from bus data.
    // Users cancel holds via the Per Schedule preset.
  }

  publish_state();
}

void InfinitESPClimate::on_system_mode_commanded(uint8_t sys) {
  // Called by the parent's set_system_mode() when ANY source (this zone's
  // control(), another zone's, or ASCII MODE!) changes the global system mode.
  // The commanding zone already set sys_mode_ in control(), so the assignment
  // below is a no-op for it; for sibling zones it updates mode + setpoints in
  // lockstep rather than waiting for the lagging bus confirm (which the
  // can_update_mode gate would defer until the next idle frame).
  //
  // Always arm the pending-mode window — even for the commanding zone — so a
  // stale AUTO-direction nibble arriving before the bus confirms can't revert
  // the just-commanded mode via the mode-trust branch.
  pending_mode_ = sys;
  pending_mode_active_ = true;
  pending_mode_until_ms_ = millis() + PENDING_MODE_WINDOW_MS;
  if (sys == sys_mode_)
    return;
  sys_mode_ = sys;
  switch (sys) {
    case SYSMODE_HEAT:  this->mode = climate::CLIMATE_MODE_HEAT; break;
    case SYSMODE_COOL:  this->mode = climate::CLIMATE_MODE_COOL; break;
    case SYSMODE_AUTO:  this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;
    case SYSMODE_EHEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
    case SYSMODE_OFF:
    default:            this->mode = climate::CLIMATE_MODE_OFF; break;
  }
  // Two-point entity: low/high are the source of truth (never the
  // target_temperature union alias of low). Each zone uses its own setpoints.
  this->target_temperature_low = parent_->setpoint_to_celsius(heat_sp_);
  this->target_temperature_high = parent_->setpoint_to_celsius(cool_sp_);
  ESP_LOGD("InfinitESP", "Zone %d: system mode broadcast -> %d", zone_, sys);
  publish_state();
}

void InfinitESPClimate::set_pending_setpoint_(uint8_t heat, uint8_t cool) {
  pending_heat_ = heat;
  pending_cool_ = cool;
  pending_active_ = true;
  pending_until_ms_ = millis() + PENDING_SETPOINT_WINDOW_MS;
  ESP_LOGD("InfinitESP", "Zone %d: pending setpoint overlay ht=%d cl=%d for %dms",
           zone_, heat, cool, PENDING_SETPOINT_WINDOW_MS);
}

bool InfinitESPClimate::compute_action_() {
  // stage>0 means the system has active demand (Carrier SAM spec). Gate
  // per-zone on the damper: a closed damper means this zone isn't receiving
  // conditioned air even while the system runs. With no zone controller,
  // zone_damper_open() is always true (single-zone system, action tracks
  // the system 1:1).
  //
  // Direction (two-layer design):
  //  1. mode nibble HEAT/COOL/EHEAT → trust it. This covers furnace heating
  //     (a gas furnace is conventional 2-stage → nibble flips to HEAT during
  //     active heat). No inference needed.
  //  2. mode nibble AUTO → read the ODU's own run-status register (0602 byte0
  //     low nibble: 2=cool, 3=heat), cached in last_odu_dir_ and confirmed by a
  //     heat-vs-cool bus diff. On variable-speed systems the nibble stays AUTO
  //     during active operation, so this is the path that resolves the old
  //     "always IDLE" bug (issue #7). Preferred over inferring from zone demand
  //     vs setpoints: it is what the equipment is actually doing, so it stays
  //     correct inside the thermostat's hysteresis band where demand-based
  //     inference reads deadband and misreports IDLE.
  //  3. else (no ODU direction yet, e.g. a fresh cycle) → IDLE. Never guess.
  climate::ClimateAction action = climate::CLIMATE_ACTION_IDLE;
  if (last_stage_ > 0 && parent_->zone_damper_open(zone_)) {
    switch (last_mode_) {
      case SYSMODE_HEAT:  action = climate::CLIMATE_ACTION_HEATING; break;
      case SYSMODE_COOL:  action = climate::CLIMATE_ACTION_COOLING; break;
      case SYSMODE_EHEAT: action = climate::CLIMATE_ACTION_HEATING; break;
      default:
        // AUTO during stage>0: the 3B02 mode nibble stays AUTO on variable-speed
        // equipment (issue #7), so resolve direction from the ODU's own run-status
        // register (0602 byte0 low nibble), confirmed by heat-vs-cool bus diff.
        if (last_odu_dir_ == ODU_RUN_HEAT)      action = climate::CLIMATE_ACTION_HEATING;
        else if (last_odu_dir_ == ODU_RUN_COOL) action = climate::CLIMATE_ACTION_COOLING;
        break;
    }
  }
  if (action != current_action_) {
    current_action_ = action;
    this->action = action;
    return true;
  }
  return false;
}

void InfinitESPClimate::on_register_update(uint8_t device_addr, uint16_t register_key) {
  bool changed = false;

  if (register_key == REG_SAM_STATE) {
    auto *data = parent_->get_register(parent_->get_sam_address(), REG_SAM_STATE);
    if (data && data->size() >= REG3B02_STAGMODE + 1) {
      uint8_t idx = zone_ - 1;
      uint8_t active = data->at(REG3B02_ACTIVE_ZONES);
      if (!(active & (1 << idx)))
        return;

      // 0xFF is the "no sensor / not ready" sentinel (reported transiently for
      // all zones during the thermostat's post-reboot warmup). Decoding it raw
      // yields ~124°C; skip the update and hold the last value, matching the
      // sensor platform's handling of the same 3B02 fields.
      uint8_t temp_raw = data->at(REG3B02_TEMPS + idx);
      if (temp_raw != 0xFF) {
        float temp = parent_->bus_temp_to_celsius((float) temp_raw);
        if (temp != current_temp_) {
          current_temp_ = temp;
          this->current_temperature = temp;
          changed = true;
        }
      }

      uint8_t stagmode = data->at(REG3B02_STAGMODE);
      // 0xFF is the same warmup sentinel the zone-temp fields guard above. Decoding
      // it raw gives stage=15 / mode=15, which falls through the mode switch to
      // CLIMATE_MODE_OFF and gets latched as a phantom "Off". Drop the whole
      // stage/mode update for a sentinel frame and hold the last good values.
      if (stagmode != 0xFF) {
        uint8_t mode = stagmode & 0x0F;
        uint8_t stage = (stagmode >> 4) & 0x0F;

        // Cache the raw stage/mode nibbles so action can be recomputed when a ZC
        // damper update arrives without a new 3B02 frame.
        last_stage_ = stage;
        last_mode_ = mode;
        // Invalidate the cached ODU direction when the system goes idle. last_odu_dir_
        // only refreshes when a 0602 frame arrives, far less often than 3B02 updates.
        // Without this, a new cycle (stage 0→>0 in AUTO) inherits the PREVIOUS cycle's
        // direction and briefly shows e.g. Heating while actually Cooling. Clearing on
        // idle makes a fresh cycle show IDLE until the real direction is confirmed.
        if (stage == 0)
          last_odu_dir_ = 0;
        if (compute_action_())
          changed = true;

        // The 3B02 mode nibble is overloaded: at stage==0 it is the requested
        // POLICY (heat/cool/heat_cool/off); at stage>0 some controls rewrite it to
        // the active DIRECTION. On this variable-speed equipment it stays AUTO even
        // while actively cooling, and direction is resolved separately from the ODU
        // 0602 register (issue #7). But Touch controls DO rewrite it to heat/cool
        // during active 2-stage operation, while legacy UIZ controls keep AUTO
        // mid-cycle on conventional 2-stage gear (issue #11). So trust any in-range
        // reading — including AUTO during stage>0, which the old logic skipped,
        // leaving a stale mode (e.g. a sentinel-induced "Off") stuck until the
        // system next idled — except in the ONE case where trusting it would flap
        // the policy heat_cool→cool→heat_cool: an already-established AUTO policy
        // seeing a HEAT/COOL direction nibble at stage>0. Reject out-of-range
        // nibbles rather than mapping them to OFF.
        //   - stage==0: always trust (nibble == policy)
        //   - stage>0 + AUTO nibble: trust (variable-speed, issue #7; unambiguous)
        //   - stage>0 + HEAT/COOL/OFF nibble, sys_mode_ != AUTO: trust (direction == policy)
        //   - stage>0 + HEAT/COOL nibble, sys_mode_ == AUTO: suppress (would flap)
        //
        // Pending mode overlay: after the user picks a mode, hold it until the
        // thermostat confirms (a poll whose nibble matches) or the window expires.
        // This stops an in-flight poll — or the parallel "System Mode" select
        // writing the same register — from bouncing the mode right after it's set.
        bool can_update_mode = false;
        // Signed-difference: a plain `millis() < until_ms` reads as expired
        // across the millis() rollover, dropping the overlay and letting the
        // bus snap the mode back the instant the user sets it.
        if (pending_mode_active_ && (int32_t) (millis() - pending_mode_until_ms_) < 0) {
          if (mode == pending_mode_) {
            pending_mode_active_ = false;  // thermostat adopted our request
            can_update_mode = true;
          }
          // else: still waiting — don't revert the user's choice
        } else {
          pending_mode_active_ = false;    // window expired (or none) — trust the bus
          can_update_mode = true;
        }
        // Issue #11: a stage>0 direction nibble must never overwrite an already
        // established AUTO policy, or HA flaps heat_cool→cool→heat_cool.
        if (stage > 0 && mode != SYSMODE_AUTO && sys_mode_ == SYSMODE_AUTO)
          can_update_mode = false;

        if (can_update_mode && mode <= SYSMODE_OFF && mode != sys_mode_) {
          sys_mode_ = mode;
          switch (mode) {
            case SYSMODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
            case SYSMODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
            case SYSMODE_AUTO: this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;
            case SYSMODE_EHEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
            case SYSMODE_OFF:
            default: this->mode = climate::CLIMATE_MODE_OFF; break;
          }
          // Update setpoints based on mode: single target for heat/cool, dual for heat_cool
          float heat_c = parent_->setpoint_to_celsius(heat_sp_);
          float cool_c = parent_->setpoint_to_celsius(cool_sp_);
          // low/high are the source of truth for this two-point entity; never
          // write the target_temperature union alias (it overlaps low, so writing
          // it in cool mode clobbers the heat setpoint).
          this->target_temperature_low = heat_c;
          this->target_temperature_high = cool_c;
          changed = true;
        }
      }
    }
  }

  // ODU run-status (0602): byte0 low nibble carries the true operating direction
  // (2=cool, 3=heat) even when the 3B02 mode nibble stays AUTO on variable-speed
  // equipment (issue #7). Cache it so compute_action_() can resolve HEATING/COOLING
  // during AUTO+stage>0. Delivered with device_addr = ODU address (any class-5 unit).
  if (register_key == REG_ODU_RUN_STATUS) {
    auto *data = parent_->get_register(device_addr, REG_ODU_RUN_STATUS);
    if (data && !data->empty()) {
      uint8_t raw = data->at(0);
      // Skip transient frames (0x10 set): during a steady cycle the ODU
      // intermittently reports the opposite direction with this bit set
      // (e.g. 0x53 = heat+transient seen mid-cooling), which would flap the
      // action to Heating until the next steady frame corrects it. Steady
      // frames (0x42/0x43) are always consistent with the real cycle.
      if (!(raw & ODU_RUN_TRANSIENT)) {
        uint8_t dir = raw & 0x0F;
        if (dir != last_odu_dir_) {
          last_odu_dir_ = dir;
          if (compute_action_())
            changed = true;
        }
      }
    }
  }

  if (register_key == REG_SAM_ZONES) {
    auto *data = parent_->get_register(parent_->get_sam_address(), REG_SAM_ZONES);
    if (data && data->size() >= REG3B03_COOL_SETPOINTS + 8) {
      uint8_t idx = zone_ - 1;
      uint8_t active = data->at(REG3B03_ACTIVE_ZONES);
      if (!(active & (1 << idx)))
        return;

      uint8_t new_heat = data->at(REG3B03_HEAT_SETPOINTS + idx);
      uint8_t new_cool = data->at(REG3B03_COOL_SETPOINTS + idx);
      uint8_t new_fan = data->at(REG3B03_FAN_MODES + idx);

      // Pending setpoint overlay: after a write, suppress stale poll data
      // for a window to prevent HA snapback while the thermostat processes the change.
      // Signed-difference: see the mode overlay above — a bare comparison
      // against the wrapping deadline discards the overlay at rollover.
      if (pending_active_ && (int32_t) (millis() - pending_until_ms_) < 0) {
        if (new_heat != pending_heat_)
          new_heat = pending_heat_;
        if (new_cool != pending_cool_)
          new_cool = pending_cool_;
      } else {
        pending_active_ = false;  // window expired
      }

      // Track whether cached setpoints changed (for publish gating)
      bool sp_changed = false;
      if (new_heat != heat_sp_) {
        heat_sp_ = new_heat;
        sp_changed = true;
      }
      if (new_cool != cool_sp_) {
        cool_sp_ = new_cool;
        sp_changed = true;
      }

      // Always update target temperatures from bus data.
      // On first boot, target_temperature_low/high are NaN even though the bus
      // values may match our defaults (68/76). We must set them unconditionally.
      float heat_c = parent_->setpoint_to_celsius(new_heat);
      float cool_c = parent_->setpoint_to_celsius(new_cool);
      if (std::isnan(this->target_temperature_low) || std::isnan(this->target_temperature_high))
        sp_changed = true;  // force publish to initialize HA state
      // low/high are the source of truth; never write target_temperature
      // (union alias of low — writing it in cool mode clobbers the heat sp).
      this->target_temperature_low = heat_c;
      this->target_temperature_high = cool_c;

      if (sp_changed)
        changed = true;
      if (new_fan != fan_mode_) {
        fan_mode_ = new_fan;
        switch (new_fan) {
          case FAN_AUTO: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
          case FAN_LOW:  this->fan_mode = climate::CLIMATE_FAN_LOW; break;
          case FAN_MED:  this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
          case FAN_HIGH: this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
        }
        changed = true;
      }

      // Infer current preset from setpoints+fan matching against comfort profiles.
      // Always run — not gated on hold_duration change — so preset reflects current
      // schedule activity even when no hold is active.
      bool hold_changed = false;
      uint16_t hold_dur = parent_->get_zone_hold_duration(zone_);
      ESP_LOGD("InfinitESP", "Zone %d hold: zones_holding=0x%02X duration=%d",
               zone_, data->at(REG3B03_ZONES_HOLDING), hold_dur);
      if (hold_dur != hold_duration_) {
        hold_duration_ = hold_dur;
        hold_changed = true;
      }

      // Determine preset display.
      // Priority: if hold is active, show hold preset. Otherwise, match
      // setpoints+fan against comfort profiles to infer activity.
      auto old_custom = this->get_custom_preset();
      auto old_preset = this->preset;

      // Vacation is the highest-priority preset: a system-wide override where the
      // thermostat forces every zone's setpoints to register 4012's min/max. 4012
      // itself only carries CONFIG (it reads identically whether vacation is
      // active or not), so the reliable active signal is the setpoint MATCH:
      // heat==4012[0] && cool==4012[1]. Confirmed on hardware: vacation ON → all
      // zones heat/cool == 4012 min/max; OFF → setpoints return to schedule.
      // 4012 is thermostat-internal and only fetched via slow-poll when emulating
      // the SAM, so in pure-passive mode vac is null and this is a harmless no-op.
      bool vacation_active = false;
      auto *vac = parent_->get_register(ADDR_THERMOSTAT, REG_TSTAT_VACATION);
      if (vac && vac->size() >= 2) {
        uint8_t vac_min = (*vac)[0];  // heat setpoint, same bus encoding as 3B03
        uint8_t vac_max = (*vac)[1];  // cool setpoint
        // 0xFF = unconfigured vacation; only match real configured values
        if (vac_min != 0xFF && vac_max != 0xFF && vac_min <= vac_max &&
            new_heat == vac_min && new_cool == vac_max) {
          vacation_active = true;
          last_activity_ = NO_ACTIVITY;
          hold_end_time_.clear();
          this->set_custom_preset_(PRESET_VACATION);
        }
      }

      if (!vacation_active && hold_duration_ > 0) {
        // Hold is active — show hold preset and compute end time
        if (hold_duration_ >= InfinitESPComponent::HOLD_PERMANENT) {
          this->set_custom_preset_(PRESET_HOLD_PERM);
          hold_end_time_ = "Permanent";
        } else {
          this->set_custom_preset_(PRESET_HOLD_TIMED);
          std::string end = parent_->format_hold_end(hold_duration_);
          if (!end.empty())
            hold_end_time_ = end;
        }
      } else if (!vacation_active) {
        hold_end_time_.clear();
        // No hold — match setpoints+fan against this zone's comfort profile row
        // (400A+zone-1). Table 40 is per-zone; matching against zone 1's row
        // (issue #23) made every other zone fall through to "Per Schedule".
        uint16_t comfort_reg = comfort_reg_for_zone(zone_);
        auto *comfort = parent_->get_register(ADDR_THERMOSTAT, comfort_reg);
        ESP_LOGD("InfinitESP", "Zone %d preset match (reg %04X): heat=%d cool=%d fan=%d comfort=%p size=%d",
                 zone_, comfort_reg, new_heat, new_cool, new_fan,
                 comfort ? (void*)comfort : nullptr,
                 comfort ? (int)comfort->size() : -1);
        bool matched = false;
        if (comfort && comfort->size() >= COMFORT_ACTIVITY_COUNT * COMFORT_ENTRY_SIZE) {
          for (uint8_t a = 0; a < COMFORT_ACTIVITY_COUNT; a++) {
            uint8_t base = a * COMFORT_ENTRY_SIZE;
            // Comfort profiles use different encoding than 3B03 setpoints:
            // °F mode: both are whole °F — direct comparison
            // °C mode: comfort = half-degrees, setpoints = whole °C
            //   Convert both to °C for comparison
            float ht_c = parent_->comfort_byte_to_celsius((*comfort)[base + 0]);
            float cl_c = parent_->comfort_byte_to_celsius((*comfort)[base + 1]);
            float sp_ht_c = parent_->setpoint_to_celsius(new_heat);
            float sp_cl_c = parent_->setpoint_to_celsius(new_cool);
            // Compare as °C with tolerance for half-degree rounding
            bool ht_match = (fabsf(ht_c - sp_ht_c) < 0.3f);
            bool cl_match = (fabsf(cl_c - sp_cl_c) < 0.3f);
            if (ht_match && cl_match && (*comfort)[base + 2] == new_fan) {
              last_activity_ = a;
              switch (a) {
                case COMFORT_HOME:  this->set_preset_(climate::CLIMATE_PRESET_HOME); break;
                case COMFORT_AWAY:  this->set_preset_(climate::CLIMATE_PRESET_AWAY); break;
                case COMFORT_SLEEP: this->set_preset_(climate::CLIMATE_PRESET_SLEEP); break;
                case COMFORT_WAKE:  this->set_custom_preset_(PRESET_WAKE); break;
                default:
                  this->set_custom_preset_(PRESET_SCHEDULE);
                  break;
              }
              matched = true;
              break;
            }
          }
        }
        if (!matched) {
          last_activity_ = NO_ACTIVITY;
          this->set_custom_preset_(PRESET_SCHEDULE);
        }
      }

      // Only publish if something actually changed
      bool preset_changed = (this->preset != old_preset) ||
                            (this->get_custom_preset() != old_custom);
      if (sp_changed || hold_changed || preset_changed) {
        changed = true;
      }
    }
  }

  // ZC damper changes can flip this zone's action without a new 3B02 frame
  // (e.g. the thermostat closes this zone's damper while the ODU keeps running
  // for another zone). Recompute from cached stage/mode + fresh damper state.
  if (register_key == REG_ZC_DAMPER_CMD || register_key == REG_ZC_ZONE_CONFIG) {
    if (compute_action_())
      changed = true;
  }

  if (changed) {
    publish_state();
  }
}

} // namespace infinitesp
} // namespace esphome
