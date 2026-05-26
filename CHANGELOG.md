<!-- markdownlint-disable MD024 -->
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Version bumps follow the project rule that A2A / MCP / schema contract changes
require a minor bump. The GPS Trust API request body is a schema contract.

## [Unreleased]

### Changed

- `rxm_sfrbx` stall-warn threshold is now configurable via the
  `sfrbx_warn_threshold` node parameter (default 100). The previous fixed
  threshold of 20 misread steady-state operation as a stall when all 6
  constellations are enabled with multi-frequency tracking at
  `CFG_RATE_MEAS=2000` — typical batch sizes are ~30-50 records.

## [0.1.0] - 2026-05-26

### Added

- `gps_trust_node` now subscribes to four additional UBX messages and ships them
  to the GPS Trust API alongside the existing `llh`, `sec_sig`, `nav_orb`,
  `nav_sat`, and `rxm_rawx` sections. New top-level request keys:
  - `nav_clock` (UBX-NAV-CLOCK) — `itow`, `clk_b`, `clk_d`, `t_acc`, `f_acc`
  - `nav_eoe` (UBX-NAV-EOE) — `itow` only; end-of-epoch fence
  - `nav_timeutc` (UBX-NAV-TIMEUTC) — calendar UTC, validity flags, `utc_std`
  - `nav_status` (UBX-NAV-STATUS) — `gps_fix`, RTK `carr_soln`, `psm`,
    `spoof_det`, `map_matching`, `ttff`, `msss`, and the navigation flags
  - `rxm_sfrbxes` (UBX-RXM-SFRBX) — **accumulated array** of subframe records
    between LLH epochs; each record carries `gnss_id`, `sv_id`, `sig_id`,
    `freq_id`, `num_words`, `chn`, `version`, and the `dwrd[]` data words
- `payloads_mutex_` guards all nine `*_json_` payload pointers. Subscriber
  callbacks build their JSON outside the lock and only take it for the
  pointer swap (or single array append, for sfrbx). The LLH drain takes the
  lock once to `std::move` all snapshots out, then serializes outside the
  lock so subscribers can keep accumulating the next batch concurrently.
- Throttled warning when the `rxm_sfrbxes` accumulator exceeds 20 records
  between LLH epochs — indicates an LLH-side stall (1 Hz max under normal
  operation).
- Per-record debug logging for `rxm_sfrbx` (was missing; other callbacks
  already logged their JSON content).
- Launch files (`ublox_gt_hpposllh_satellite.launch.py` and the X20P variant)
  now enable `CFG_MSGOUT_UBX_NAV_CLOCK_USB`, `CFG_MSGOUT_UBX_NAV_EOE_USB`,
  and `CFG_MSGOUT_UBX_NAV_TIMEUTC_USB` for the F9P configuration. The X20P
  launch gains the same three (the F9P launch already had NAV_STATUS and
  RXM_SFRBX enabled).

### Changed

- `ubx_nav_llh_callback` replaces the per-payload `use_count()` /
  `*.get()` / `.reset()` sequence with a single-lock snapshot followed by
  conditional serialization outside the lock. Drained sections are absent
  from the request when no callback fired between epochs (no nulls on the
  wire).
- `dwrd[]` values are explicitly cast to `Json::UInt` before append to
  remove any overload-resolution ambiguity. Encoding is unchanged
  (decimal uint32 in the `[0, 2^32-1]` range; JSON-safe).
- Recent (already-released) contract change: action-param routing now
  strips `NtripClient_` / `UbxCfg_` prefixes, resolves `node_type` from
  explicit `ntypes` or prefix, and falls back to the parameter cache when
  neither is present.
- Service-wait now requires both NTRIP and u-blox parameter clients before
  initialisation, with a 30-attempt timeout fallback.

### Fixed

- Empty NTRIP / UBX parameter loads now log a warning instead of silently
  proceeding.

## [0.0.1] - 2025-01-28

### Added

- Initial release: `gps_trust_node`, `gps_trust_msgs`, `gps_trust` launch
  package. Posts `llh`, `sec_sig`, `nav_orb`, `nav_sat`, `rxm_rawx`, and
  parameter-cache snapshots to the GPS Trust API and republishes the
  returned trust level as a `GPSTrustIndicator` message.
