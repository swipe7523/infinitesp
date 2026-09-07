# InfinitESP — Code Review Findings

**Scope:** all ~8,500 lines of C++/Python/YAML in `components/` (`infinitesp`, `uart_rmtx`,
`sam_ascii`), reviewed as 24 units per `PLAN.MD`.
**Date:** 2026-09-06 · **Base:** branch `dev` at `dcabad1` (+ the fix to finding #1)
**Scope limit:** correctness only — memory safety, protocol framing, arithmetic, state
machine, off-by-one, unit conversion, resource growth. Style and architecture excluded.

**31 findings: 1 High · 11 Medium · 19 Low.**

**Fix pass — 27 of 31 fixed.** Every finding is now fixed except four, each left open
deliberately: **#22** (needs metric hardware), **#10** and **#15** (need a judgment call
that is better made by the maintainer), and **#16** (settled as benign). Fixes are marked
inline below; each was verified by a local compile plus, where the defect was demonstrable,
the same harness that found it.

| Severity | Meaning |
|---|---|
| **H** | Wrong control output, crash, or memory unsafety |
| **M** | Wrong reported value, or a state needing a reboot to clear |
| **L** | Cosmetic, latent, or defensive |

Every finding names a concrete failure scenario. Where a defect exists but no caller can
reach it, that is stated explicitly rather than inflated.

---

## 1. Fixed

### #1 · H · `infinitesp.h:342-347` — the branch did not compile
Six ODU register constants (`REG_ODU_COMP_SPEED`, `_DEMAND`, `_CMD_STAGE`, `_STAGE_INFO`,
`_SETPOINT`, `_FLOATS`) were defined twice at namespace scope. Introduced by merge
`42b8d3f`, which kept both sides of the block — each parent has one copy, the merge has two.

Namespace-scope `static const` redefinition is a hard error, so **no ESPHome build of this
component could succeed**. Fixed by deleting the stale copy at 342-347; the surviving copy
at 323-328 is value-identical (only the `REG_ODU_FLOATS` comment differs, and the survivor
has the newer text). Verified by a full `esphome compile` — success, zero warnings.

---

## 2. Live bugs — recommended fix order

Each is small, independently verifiable with the local compile, and reachable in normal use.

### #13 · M · `infinitesp.cpp:1208-1212` — dead code disables the authoritative °C/°F flag

**Status: FIXED.** The block now sits outside the 3B02/3B03 branch (reachability re-verified by brace-depth check), and the WRITE path its own comment describes was added to `handle_write_request_`.
The `if (sam_enabled() && reg_key == REG_SAM_DEALER ...)` block sits at brace depth 1 inside
`if (reg_key == REG_SAM_STATE || reg_key == REG_SAM_ZONES) {` (opens 1156, closes 1213 —
verified by brace counting). `REG_SAM_DEALER` is `0x3B06`; the enclosing guard admits only
`0x3B02`/`0x3B03`, so the condition is **unsatisfiable**.

It is also in the wrong handler: its own comment says the thermostat *pushes* 3B06, and a
push is a WRITE, which routes to `handle_write_request_`, never to `handle_reply_`.

**Impact:** the only other caller (1412) is the 3B05 FakeSAM path, which `loop()` gates on
`!sam_enabled()` (405). So whenever the component emulates a SAM — the normal setup —
`metric_units_known_` never becomes true and unit detection silently falls back to the
"active zone temp ≤ 50 → °C" heuristic instead of the verified flag. This reaches past zone
temps: in °C mode comfort bytes are half-degrees, so a wrong unit decision scales comfort
setpoints by 2× (see #22).

### #14 · M · `infinitesp.cpp:1300-1303` — `outdoor_unit_address:` is silently ineffective

**Status: FIXED.** Now filters with `is_odu_addr()`, honoring the learned address and any YAML override; falls back to the nibble only before discovery, so pre-discovery behavior is unchanged.
`poll_odu_slow_` picks targets with the raw address-nibble convention,
`if ((akv.first >> 4) == CLASS_OUTDOOR_UNIT ...)`, instead of `is_odu_addr()`. It therefore
ignores both the discovered `odu_address_` and any YAML override — the exact assumption
`learn_device_` exists to replace.

**Impact:** an ODU outside class 5 is never slow-polled, so its acquisition-backstop
registers (0304 line voltage; 3E01/3E02/3E08 on 2-stage units) are never acquired and those
sensors stay permanently unknown. A user setting `outdoor_unit_address:` sees it honored
everywhere else and skipped here. Not hypothetical class drift: **this install has its IDU
at 0x3E (class 3)**, so the convention already fails for one role.
`notify_entities_` 30 lines below does this correctly — an isolated miss.
**Fix:** `is_odu_addr(akv.first)`.

### #25 · M · `infinitesp.cpp:2483-2498` — stack over-read into a publicly-pasted report

**Status: FIXED.** Added a clamping `emit_buf(n)` helper and routed all 8 sites through it; it also discards a negative encoding error.
`n = snprintf(buf, sizeof(buf), ...)` returns the length it *would* have written; on
truncation `n > sizeof(buf)` and the following `write_fn(buf, n, ctx)` reads past the
256-byte **stack** buffer. All 8 sites in `stream_bus_report_` use this pattern; seven have
short fixed formats, but the report header interpolates ten `uint32` counters.

**Impact:** measured worst case **289 bytes from a 256-byte buffer** — `snprintf` writes
255 + NUL, returns 289, and `write_fn` emits **33 bytes of adjacent stack memory** into the
report. A realistic 30-day uptime already reaches 226 bytes, leaving ~30 bytes of headroom
while `rx` grows ~10/s. This is the `REPORT?` output users paste into issues.
**Fix:** clamp — `if (n > (int) sizeof(buf)) n = sizeof(buf) - 1;` at every site.

### #30 · M · `infinitesp/__init__.py:84` — no address collision checking exists

**Status: FIXED.** `_validate_addresses` now rejects collisions between the SAM, the ZC, the implicit secondary (`zc + 1`), the thermostat's own 0x20, and the IDU/ODU overrides. Verified against real configs: all three collision shapes rejected, valid configs still pass.
`_validate_addresses` handles only the `address` → `sam_address` deprecation and a default.
All four address options are `cv.int_range(min=0, max=255)` with **no cross-validation**,
and the secondary ZC address is **implicit** (`zc_address_ + 1`), so it is never checked
against anything.

**Impact:** `zone_controller_address: 0x92` silently collides with the default
`sam_address`. `initialize_defaults_` seeds the SAM block and then the ZC block into the
*same* `device_registers_[0x92]`, so the ZC's 0104 ("INFINITESP ZONE CTRL") overwrites the
SAM's ("SYSTEM ACCESS MODULE") — the thermostat reading 0104 from the SAM gets a
zone-controller nameplate and enrollment breaks. The subtler case needs no obvious mistake:
`zone_controller_address: 0x91` with the default SAM at 0x92 collides **via the implicit
secondary**, so a user who carefully checks ZC ≠ SAM still loses.

### #27 · M · `infinitesp.cpp:2771, 2797` — a sentinel filter that exists elsewhere but not here

**Status: FIXED.** Filters the 0x00/0xFF sentinels; when zone 1 is itself unusable the TLV entry is left untouched (thermostat holds its last plausible value) rather than written as not-installed, which would risk re-commissioning the zone.
`check_zc_sensor_fallback_` takes zone-1 ambient straight from `(*state)[REG3B02_TEMPS]`
with no sentinel filter, then publishes it as a **present** zone temperature for every
stale-sensor zone. The 3B02 unit heuristic 1500 lines earlier filters exactly this byte, and
its comment says why: `0x00` is "common during thermostat reboot".

**Impact:** if a configured zone sensor is stale *and* the thermostat's 3B02 carries the
`0x00` sentinel, the fallback computes `z1_f = 0 °F` (32 °F on a °C bus) and writes it with
`present = true`. The thermostat sees zones at 0 °F and calls for maximum heat. Requires
both conditions and self-heals once 3B02 repopulates — but equipment moves meanwhile.
`0xFF` is equally unfiltered and would publish 255 °F.

### #9 · M · `infinitesp.cpp:387, 404` — a documented invariant the code doesn't implement

**Status: FIXED.** Replaced the three per-class flags with a single `initiated_tx` gate covering all six initiated-TX paths, which enforces the invariant structurally rather than requiring each new gate to repeat the terms. Gates 1-4 are semantically unchanged.
The ODU slow poll correctly gates on `!fast_poll_sent && !retransmit_sent &&
!tstat_slow_sent`, but the discovery probe (387) and metric-units probe (404) omit
`tstat_slow_sent` and don't exclude each other. Discovery's own comment claims "never in the
same iteration as a thermostat poll", which the code does not implement.

**Impact:** the 31 s thermostat slow poll and the 3.5 s discovery probe become due in the
same `loop()` iteration regularly, so two READ frames go out back-to-back with no reply
between — the pattern the slow-poll comment at 322 blames for a "44% poll timeout rate".

### #26 · M · `infinitesp.cpp:2692` — negative float → `uint16_t` is undefined behavior

**Status: FIXED.** Clamps to [0, 65535] before the cast. Verified byte-identical across the whole positive band (0 differences in 25,001 samples), so this is a pure UB fix. Encoding for negatives is deliberately left as clamp-to-zero, not two's complement — see the code comment; the wire format needs hardware to settle.
`write_zc_temp_entry_` encodes `raw = (uint16_t)(temp_f * ZC_TEMP_SCALE + 0.5f)`. The
thermistor band is deliberately `-40 .. 250 °F` ("supply air can exceed the 40-99 °F indoor
band"), so a negative `temp_f` is explicitly in-contract.

**Impact:** a LAT/HPT reading below 0 °F passes the sanity band, then the cast is UB — in
practice 0 or a large wrap — and that value is written into the ZC 0302 TLV as a *present*
leaving-air temperature the thermostat consumes. The ODU decoders treat the same physical
quantity as **`int16_t`** (`decode_int16_f_`), so the encoding itself may also be wrong
(unsigned where the wire format is signed). The zone path is unaffected (40-99 °F band).

### #31 · M · `uart_rmtx/uart_rmtx.cpp:99-106` — unchecked `malloc`, made permanent

**Status: FIXED.** Allocates before freeing, adopts the new buffer only on success, logs the failure, and `write_array` now checks the buffer before encoding. Verified by compiling `uart_rmtx` in an isolated test config (it is not built by `infinitesp.yaml`).
`ensure_sym_buf_` does not check `malloc`'s return, yet sets `sym_buf_cap_ = needed_symbols`
unconditionally, and `free()`s the existing buffer *before* attempting the new allocation.
`sym_buf_` is never null-checked at any of its three use sites.

**Impact:** on allocation failure (~10 KB internal RAM for a max-length frame, under
WiFi/API heap pressure) `sym_buf_` becomes `nullptr` while `sym_buf_cap_` stays non-zero.
`write_array` then calls `line_enc_.encode(data, len, nullptr, cap)` — a crash. It is
**persistent**: every later call takes the `sym_buf_cap_ >= needed_symbols` early return, so
the buffer is never re-allocated and the device reboot-loops.
Not built by the current `infinitesp.yaml`; affects configs using `uart_rmtx`.

### #7 / #29 · M / L — one root cause, three instances: wrapping absolute deadlines

**Status: FIXED (all three).** Now `(int32_t)(millis() - until_ms) >= 0`. Verified with a rollover simulation: the old form fires immediately at the wrap, the new one waits and then fires on time.
`millis() + WINDOW` stored, then compared with `<` or `>=`. Not rollover-safe. The codebase
uses the correct `(int32_t)(x - now)` idiom in **three other places**, so these are
oversights, not a misunderstanding.

- **#7 · M · `infinitesp.cpp:156` (+1524)** — hold-set debounce. Every ~49.7 days there is a
  `debounce_ms`-wide window (~2 s) where the comparison is immediately true, the debounce is
  skipped, the hold fires on the *first* value the user scrolled past, and the value they
  settled on is never written. **HVAC gets a hold duration the user didn't pick.**
- **#29 · L · `infinitesp_climate.cpp:339, 416`** — the two 8-second optimistic overlays.
  At rollover the overlay is discarded instantly and the UI snaps back to the stale bus
  value — the "HA snapback" the overlay exists to prevent. Display-only.

**Fix for all three:** `(int32_t)(millis() - until_ms) >= 0`.

---

## 3. Latent — held safe only by the zone perimeter

Every zone-indexed setter takes `uint8_t zone`, computes `zone - 1`, and indexes with it
**without runtime validation**: `set_zone_setpoint`, `set_zone_fan`, `set_zone_hold` /
`encode_hold_`, `apply_activity`, `get_zone_hold_duration`. `queue_hold_set` is the lone
function that does validate — the intent existed and reached one of six places.

What keeps them safe is entirely external, and is currently airtight:

- **Build time** — all eight entity platforms declare `cv.int_range(min=1, max=8)` on
  `zone` (verified in every `*/__init__.py`), so an out-of-range zone fails `esphome config`.
- **Runtime** — the only other entry, the SAM ASCII parser, accepts a zone prefix only when
  `body[1] >= '1' && body[1] <= '8'` (`sam_ascii.cpp:199`), else leaves `zone = 1`.

The consequences behind that perimeter differ sharply in kind:

| # | Sev | Function | With `zone = 0` or `9` |
|---|---|---|---|
| **17** | M | `set_zone_setpoint` / `set_zone_fan` | `zone=0` → `idx=255` → writes at offsets 258/267/272 of a 150-byte vector — **out-of-bounds heap write, 117 bytes past the end**. Zones 9-130 stay in bounds but hit the **wrong field** (zone 9's heat lands on zone 1's cool) and transmit it. |
| **19** | L | `encode_hold_` | `1 << 255` is **undefined behavior**; `hold_offset` is a `uint8_t` so 548 truncates to 36, silently clobbering `SPEED_FAN` and the `TIMED_HOLDS` bitmask for every zone. |
| **20** | L | `apply_activity` | Needs **no memory unsafety at all**: `comfort_reg_for_zone(9)` returns `0x4012` = `REG_TSTAT_VACATION`, which *is* stored (7 bytes) and *passes* the size guard, so vacation min/max/fan are read as a comfort profile and written to the thermostat. |

**A single relaxed YAML validator, or one new caller, converts all three into live bugs at
once** — including an out-of-bounds write.

**Status: ALL FIXED.** Every zone-indexed entry point (`set_zone_setpoint`, `set_zone_fan`,
`set_zone_hold`, `apply_activity`, `get_zone_hold_duration`) now validates `zone 1-8` at
runtime, matching `queue_hold_set`, so the YAML schema is no longer the only defense.
`apply_activity` additionally range-checks `activity_index`, and **#23**'s clamp is in place.

Also latent: **#23 · L** — `celsius_to_setpoint` / `celsius_to_comfort_byte` end in
`(uint8_t) roundf(...)` with no domain clamp, and float→integer out of range is UB.
ESPHome's `ClimateCall::validate_()` rejects NAN and unsupported modes but performs **no
clamping to `visual_min`/`visual_max`**, so a direct API call with
`set_target_temperature_low(200.0)` reaches `c_to_f = 392.0f` → UB. The HA UI respects the
visual limits, so this needs an API client, not a normal user action.

---

## 4. Unresolved — needs metric hardware

### #22 · M · `infinitesp.cpp:2186` vs `1832` — two comments contradict each other
`comfort_byte_to_celsius` implements the header's claim ("In °C mode, comfort bytes are
half-degrees, byte/2 = °C"). `apply_activity` states the opposite as a known defect:
"comfort_byte_to_celsius() is itself buggy in °C mode (400A is always °F, not C*2)".
One of them is wrong.

If `apply_activity` is right, then on a **metric install** a comfort byte of 68 (≈20 °C)
decodes as `68/2 = 34.0 °C`, and `celsius_to_setpoint` re-encodes it as a whole-degree
**34 °C ≈ 93 °F** written to the HVAC — applying the "home" activity would command 34 °C
instead of 20 °C. If the header is right, there is no bug.

This compounds with **#13**: because the authoritative metric flag is dead code in the
SAM-emulated path, a metric system reaches this branch via the temperature heuristic, which
*correctly* detects °C — so the heuristic working as designed is precisely what would
trigger the wrong branch. **This install is °F and unaffected.** Highest-value item to
verify on real metric hardware.

---

## 5. Remaining low-severity findings

| # | Location | Note |
|---|---|---|
| 2 | `infinitesp.h:117` | `fault_entry_time_valid` carries no size guard of its own and only its sibling documents the precondition. **Both callers guard correctly** (`>= FAULT_REG_SIZE`), so not live — a trap for a third caller. Downgraded from H? after checking. |
| 3 | `infinitesp.h:87` | `comfort_reg_for_zone` has no range check; `zone=0` → `0x4009`, `zone=9` → vacation register. See #20. |
| 4 | `infinitesp.h:141` | `REG_TSTAT_FAULTS` comment says 70 bytes while `FAULT_REG_SIZE = 72` correctly includes the day-counter trailer. Cosmetic; the trailer is load-bearing. |
| 5 | `infinitesp.h:813` | `odu_float_(data, 0)` computes offset `-3` → `size_t` 2⁶⁴−3, and `decode_f32_be_`'s `offset + 4 > size()` guard **wraps and passes**. Verified with a repro. No caller can reach it, but all three shared decode helpers present a guard that silently fails for any negative offset. |
| 6 | `infinitesp.h:1026` | The `TrafficKey` "footgun" comment describes a lossy packed key and recommends a struct — the code below **already is** that struct. Stale comment; **no collision exists**. Delete it. |
| 8 | `infinitesp.cpp:1528` | `get_zone_hold_duration` has no zone range check; zones 9-16 read zone-*name* bytes as a duration, `zone >= 33` makes `1 << idx` UB. |
| 10 | `infinitesp.cpp:718` | `learn_device_` classifies on bare substrings (`has("COMP")` matches COMPANY/COMPACT) and latches with no debounce. Two matching devices make `odu_address_` flip on every 0104 reply. Note the *absence* of a bus-class guard is correct and deliberate — IDU 0x3E is class 3. |
| 11 | `infinitesp.cpp:955` | `frame.push_back(payload.size())` truncates `size_t` → `uint8_t` unguarded. Largest reply lands on exactly 255 — the boundary, zero margin. |
| 12 | `infinitesp.cpp:1045` | 0104 log guards `size() >= 96` then reads a serial at `[96]` with precision 24 (**needs 120**). Both defaults are exactly 120 bytes and 0104 is write-protected, so not triggerable; guard should be `>= 120`. |
| 15 | `infinitesp.cpp:1478` | `store_register_` has **no cap, no eviction** — the one genuinely unbounded structure. The author capped `traffic_log_` (256, LRU) with a comment naming this exact scenario, but not the store, which holds far larger values. Slow: CRC16 admits ~1 in 65536 corrupt frames. |
| 16 | `infinitesp.cpp:1478` | No length validation, so a short reply overwrites a good cached register. **Benign outcome**: decoders return without publishing and the final publish is gated on `!isnan`, so sensors hold their last value rather than publish garbage. Silent staleness, not a wrong reading. |
| 18 | `infinitesp.cpp:1624` | No deadband/ordering validation. ESPHome rejects `low > high` in **one** call, but two sequential single-bound calls assemble an inverted pair, and the ASCII paths bypass it entirely. |
| 21 | `infinitesp.cpp:1888` | `set_system_mode` doesn't range-check `mode` (valid 0-4), only masks `& 0x0F`. Unreachable: ASCII rejects unknown names via the `0xFF` sentinel and climate maps only known modes. |
| 24 | `infinitesp.cpp:2311` | On a **monochrome** status light, "WiFi not connected" renders **solid** — indistinguishable from all-good. Its 250 ms blink phase is sampled at a 500 ms throttle; simulated: **0 flips in 20 s**. RGB and GPIO-pin modes are fine. |
| 28 | `infinitesp_climate.cpp:78-96` | Dead code: the entity declares two-point support, so ESPHome always resets `target_temperature_` and this ~19-line branch cannot execute. No functional impact, but it reads as the load-bearing single-setpoint path. |

---

## 6. Checked and found sound

Roughly as useful as the findings — several of the review plan's stated suspicions turned out
to be non-issues, and two premises were simply wrong.

- **Unchecked register indexing (B1)** — all 24 IDU/ODU accessors checked mechanically.
  Every guard matches its highest index **exactly**; several to the byte
  (`REG_ODU_FLOATS` needs `>= 25` and reads through byte 24). `handle_passive_frame_`,
  `dispatch_frame_`, `handle_reply_` and the generic sensor decoder are all clean.
- **Key collisions (B6)** — **none exist anywhere.** `TrafficKey` is a full four-field
  struct with lexicographic ordering; the register store is a two-level
  `map<addr, map<key, data>>`, so `(addr, key)` cannot alias across devices.
- **Frame parser (B4)** — transcribed 1:1 and fuzzed under ASan+UBSan: all 256 payload
  lengths round-trip, **a corrupt frame followed by a valid one recovered 2000/2000**,
  200k random noise bytes produced no crash and no bogus frame, and `rx_buffer_` never
  exceeded **265 bytes** (exactly the theoretical max). Loop termination is guaranteed.
- **Temperature round-trips (B7)** — the plan expected "a setpoint that drifts every time HA
  touches it". All three conversion pairs were run exhaustively over raw 0-255 in **both**
  bus modes: **0/256 mismatches everywhere**. The HA direction settles to a fixed point
  after one pass — quantization is stable, not cumulative.
- **`pad_str` overflow** — the plan called it "a classic overflow". It cannot be: it
  `push_back`s into a `std::vector` exactly `width` times, so it truncates. All 12 call
  sites checked; every field is comfortably NUL-terminated.
- **CRC symmetry** — TX and RX compute over the identical span (header + payload, length
  byte included) in the same byte order. No off-by-one.
- **`loop()` fan-out (B8)** — `notify_entities_` costs O(entities), not O(register store);
  the plan's framing inverted it. The UART drain is bounded by the 1024-byte RX buffer.
- **Credentials** — the password is **never logged**; grepping every log statement in
  `components/` for "passw" returns zero hits. The NVS copy always NUL-terminates
  (`CachedWifi cached{}` + `strncpy(..., sizeof - 1)`), and `REPORT?` deliberately skips the
  WiFi register.
- **JSON escaping** — `emit_json_string_` escapes `"`, `\`, all control bytes **and**
  everything from 0x7F up (so 0xFF / telnet IAC becomes `\u00XX`), with a bounds-safe
  batching buffer. Bus-supplied device names cannot break the report.
- **Two-controller zone mapping** — the plan's "prime off-by-one site" is correct:
  `zc_addr_for_zone_` and `zc_local_id_for_zone_` map zones 5-8 onto the secondary's local
  1-4 properly, and the system-wide damper registers correctly use a *different* mapping.
- **Chunk 21 as counter-example** — every ZC zone entry point validates `zone < 2 || zone > 8`
  at runtime, and the boot staleness check guards with a NAN test *before* the timer, so
  `last_update = 0` cannot read as fresh. This is the defensive style the Phase 3 setters
  lack, in the same file.

---

## 7. Method

No build or test loop existed locally at the start; one was established during the review.

- **Authoritative build:** `~/.venvs/esphome/bin/esphome compile infinitesp.yaml`
  (ESPHome 2026.6.5 on PlatformIO's bundled Python 3.11.7). Full build ≈276 s, zero warnings.
- **Fast loop:** `scratchpad/check.py` — replays each file's exact recorded compiler command
  with `-fsyntax-only`, ~2 s/file (~20 s for all 10 TUs). Validated by re-introducing a known
  error and confirming it fails. Note ESPHome's `compile_commands.json` omits
  `-I<build>/src` and `-DUSE_ESP32 -DUSE_ESP_IDF`, which must be added back.
- **Parser fuzzing:** `scratchpad/fuzz_parser.cpp` — 1:1 transcription of `parse_byte_` /
  `validate_frame_` / `front_header_plausible_`, built with ASan+UBSan.
- **Conversion round-trips:** `scratchpad/roundtrip.cpp` — exhaustive over the full 0-255
  domain in both bus-unit modes.
- Guard/offset audits, brace-nesting checks, and LED blink aliasing were verified with
  throwaway scripts rather than by eye.

**Not verified:** anything requiring live hardware or a metric (°C) thermostat — notably
#22, and the real-nameplate assumptions behind #10. `uart_rmtx` and `sam_ascii` are not
built by the current `infinitesp.yaml`, so #31 is static reasoning only.
