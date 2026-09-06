#include "infinitesp_cover.h"

namespace esphome {
namespace infinitesp {

cover::CoverTraits InfinitESPCover::get_traits() {
  cover::CoverTraits traits;
  traits.set_supports_position(true);
  traits.set_supports_stop(false);
  return traits;
}

void InfinitESPCover::apply_step_(uint8_t new_step) {
  if (new_step == last_step_)
    return;  // same step we last acted on — no change, no fire, no publish
  last_step_ = new_step;
  this->position = new_step / 15.0f;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->change_trigger_.trigger(this->position);
  this->publish_state();
}

void InfinitESPCover::control(const cover::CoverCall &call) {
  // HA command. ESPHome maps open→1.0, close→0.0, slider→value, all as a
  // position. The cover never writes the bus: fire the same trigger the bus
  // path uses and publish so HA acknowledges. HA-set positions are transient —
  // the next bus-commanded change overwrites them.
  const auto &pos = call.get_position();
  if (!pos.has_value())
    return;
  // Quantize to the protocol's 16 steps so a re-command of the current step
  // (e.g. HA slider at 0.53 while the cover sits at step 8 = 0.533…) does not
  // spuriously fire. Anchor: last_step_ (0xFF sentinel fires the first time).
  apply_step_((uint8_t) lroundf(*pos * 15.0f));
}

void InfinitESPCover::on_register_update(uint8_t device_addr, uint16_t register_key) {
  // Damper command is an 8-byte WRITE to 0308, one byte per system zone. We
  // read 0308, not 0319 state feedback (the secondary controller returns
  // all-FF there). Both keys notify; accept either and re-read 0308.
  if (register_key != REG_ZC_DAMPER_CMD && register_key != REG_ZC_ZONE_CONFIG)
    return;
  if (zone_ < 1 || zone_ > 8)
    return;
  // Only react to the controller serving this zone (0x60 for zones 1-4, 0x61
  // for 5-8). Both store identical system payloads and both notify; this guard
  // keeps each cover to one update per cycle.
  if (device_addr != parent_->zc_addr_for_zone_(zone_))
    return;

  // System-wide payload: zone N is at byte N-1 (zc_system_byte_for_zone_).
  auto *data = parent_->get_register(device_addr, REG_ZC_DAMPER_CMD);
  if (!data)
    return;

  // The damper byte is itself the step (0x00-0x0F). Compare in step space so
  // the 0308/0319 mirror (same value, two notifies) and the thermostat's
  // periodic re-asserts fire exactly once per real change, and so an HA-set
  // transient (control()) snapping back to the bus value still fires when the
  // steps actually differ. Anchor: last_step_ (0xFF sentinel fires the first
  // time, so a zone fully-open at boot still actuates).
  //
  // 0308/0319 are 8-byte system-wide payloads, one byte per system zone: zone N
  // sits at byte N-1, not (N-1)%4 — that local-id mapping aliases zones 5-8 onto
  // 1-4 (issue #9). zc_system_byte_for_zone_ is the named anchor for that.
  uint8_t byte_idx = parent_->zc_system_byte_for_zone_(zone_);
  if (byte_idx >= data->size())
    return;
  apply_step_((*data)[byte_idx]);
}

}  // namespace infinitesp
}  // namespace esphome
