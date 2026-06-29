# TODO

Outstanding work for InfinitESP. Because this is a reverse-engineered ESPHome
component (no app, no test suite — see `CLAUDE.md`), most open items are
**undecoded bus registers** or **decodes confirmed on only one physical system**,
not missing features. Validate every protocol change against captured traffic or
`REPORT?` output, not assumption.

## Protocol decode gaps (ODU inverter telemetry)

- [x] **Decode inverter telemetry in ODU `060a`.** DC-bus voltage (`data[98]`,
      u16 BE /16 V), AC line current (`data[102]`, u16 BE /256 A), PFCM temp
      (`data[110]`) and IPM temp (`data[112]`, both u16 BE /16 °F) decoded and
      cross-validated vs the Anantha MQTT fork over the 24h `day_1782524237`
      capture using the OFF→HIGH endpoint discriminator (median |err| ≤0.5° for
      temps, 0.03 A for current; dc-bus noisier — narrow 312–327 V range). Units
      confirmed by the Anantha field registry. (`infinitesp.h` 060a accessors)
- [ ] **Localize EXV position in `060a`.** Still undecoded: in the available
      capture the EXV is pinned at 0% (compressor off) / 100% (running), so it
      has no intermediate dynamic range to fit. Needs a capture with the EXV
      modulating (low-load / part-stage operation) to find the offset.
      (`anantha_exv_position` is the oracle metric)
- [ ] **Identify ODU `0302` slots idx 3/4.** Slots 0/1/2/5 are real °F temps,
      but idx 3/4 decode to non-temperature data (were publishing ~182/175 °C
      garbage, now band-guarded to `NAN`). Their true meaning is unknown.
      (`infinitesp.cpp:628`; `CLAUDE.md`)
- [ ] **Confirm ODU `060b` (`REG_ODU_SETPOINT`).** Target value at `byte[2]`
      in native °F — "label TBD; not confirmed a cooling setpoint."
      (`infinitesp.h` REG_ODU_SETPOINT)
- [ ] **Measure stage-5 compressor frequency.** The `0608` frequency scale is
      confirmed for stages 1–4 against Carrier rated RPM; stage 5 (144 Hz) is
      *predicted, not yet measured*. Capture a real stage-5 cycle to verify.
      (`infinitesp.h:435`, `odu_compressor_frequency_`; see `private/DEVLOG.md`
      2026-06-23)
- [ ] **Resolve the always-`0x1E` header bytes.** Frame `byte[5-6]` purpose is
      unclear (always `0x1E` on this system). (`infinitesp.h:76`)

## Validation & hardware coverage

- [ ] **Cross-system / multi-firmware validation.** Only the author's own system
      is confirmed working. Register layouts are empirically derived and may be
      wrong or system-specific. Validate against other Infinity/Evolution/ICP
      equipment and firmware revisions before treating decodes as general.
      (`README.md` → Targeted Systems, Disclaimer)
- [ ] **Exercise Zone Controller emulation / damper paths on a real ZC.** The
      author's bus has no physical ZC, so damper data and the ZC-emulation
      (`0x60`) paths are only partially exercised. Confirm cover positions and
      per-zone conditioning state on a multi-zone system with a real ZC.
      (`README.md:312`; `infinitesp.h:278`)
- [ ] **Regression-check shipped decoders periodically** with
      `scratchpad/validate_known.py` against fresh simultaneous bus + Anantha
      captures, to catch drift or system-specific assumptions.

## Notes

- No compiled-binary build, unit tests, or linter exist. The "build" is
  `esphome compile infinitesp.yaml` (this is also the typecheck).
- The full reverse-engineered register layouts live in the gitignored
  `AGENTS.md` (if present locally); decode tooling and captures live in the
  gitignored `scratchpad/` (start at `scratchpad/README.md`).
