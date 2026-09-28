# Validation record — 2026-09-28

## Developer-mode separation update

Local verification on 2026-09-28 used Xcode 26.4 / Swift 6 and iOS simulators. Ordinary Release is compile-closed at both eligibility and fixture-client construction. The dedicated TestFlight build requires verified StoreKit sandbox evidence and matching bundle identity before offering session opt-in. See [distribution policy and physical-device checklist](DEVELOPER_MODE.md); sandbox evidence alone is not TestFlight channel attestation.

Passed checks:

- **21 unit tests in Debug and 21 in Release**: production eligibility and explicit fixture-construction denial, TestFlight evidence/identity policy, session defaults and transitions, exact telemetry/catalog source checks, simulated-job checks, endpoint/auth data validation, freshness, demo model, and existing operation request/cancellation guards. Release tests use test-only `ENABLE_TESTABILITY=YES` for `@testable`; no DEBUG flag is added to Release.
- **3 Debug iPhone UI tests**: explicit opt-in, authenticated fixture readings, disconnect/local sample, real-mode launch and map state, background exit, and confirmed simulated operation start/stop. Exported telemetry and operation screenshots were visually checked; the persistent developer banner no longer overlaps navigation titles.
- **Unsigned physical-device builds** for ordinary Release and dedicated TestFlight configurations. Neither result verifies signing or installation.
- **Release iPhone UI**: no initial samples, unavailable map, no developer entrypoint. **TestFlight simulator UI**: initial tools remain absent without verified eligibility. This does not establish completed StoreKit verification or actual TestFlight receipt behavior.
- Shared Swift transport/controller in explicit Debug fixture mode: authentication, failed-auth recovery, polling pause/resume, disconnect cancellation, and populated telemetry/catalog/control clearing on return to robot mode. Delayed/stale telemetry and automatic network recovery harnesses passed.
- Managed-operations fixture: 20-operation catalog, explicit lease, confirmation, conflict/dependency rejection, retained uncertain outcomes, ordered stop-all, background lease expiry and manual reacquisition. All operations were simulated.
- Adversarial transport harness (`python3 Tests/run_gateway_mode_checks.py`): source mismatch and cancellation during preflight each caused one catalog read and **zero POSTs**; a dropped mutation response caused **exactly one POST** and no retry. It explicitly compiles Debug fixture access.
- Independent source review found no remaining actionable issues. Local setup-bundle links and `git diff --check` passed.

Current evidence (local temporary results):

- `/tmp/helios-developer-unit-verified.xcresult`
- `/tmp/helios-developer-release-unit-recovered.xcresult`
- `/tmp/helios-developer-ui-verified.xcresult`
- `/tmp/helios-developer-release-ui-iphone.xcresult`
- `/tmp/helios-developer-testflight-ui.xcresult`
- `/tmp/helios-developer-release.log` and `/tmp/helios-developer-testflight.log`

Earlier attempts exposed a virtualized offscreen Form selector and an iPad tab-selector mismatch; the iPhone UI suite corrects/avoids those selector assumptions. Visual review caught a developer banner overlapping navigation titles; its layout was moved above TabView. A simulator-service crash interrupted later runs; sequential tests after a completed reboot recovered. A first Release unit invocation lacked testability and was rerun with the test-only flag above.

No physical iPhone, signed TestFlight/App Store installation, or robot has been validated. TestFlight Internal Only upload, signed sandbox eligibility, offline/cached/missing evidence, upgrades/reinstalls, physical TLS/connectivity, ROS replay provenance and real stopping behavior remain device/deployment checks. Gateway metadata is an adapter claim, not hardware attestation.

## Initial integration record (before developer separation)

## Scope and environment

Xcode 26.4, Swift 6 language mode, iPhone 17e simulator with iOS 26.4. Companion gateway changes are in the [upstream pull request](https://github.com/pran99-git/helios_ws/pull/1). All command tests used the gateway's **simulated process backend**. No physical rover was connected, actuated or commissioned.

## Passed checks

- iOS app and test targets compile for Simulator.
- **10 unit tests** in three suites: demo playback/reset and trajectory units; endpoint/token validation; monotonic freshness including huge finite ages; invalid/versioned payloads; disconnected/demo state; typed command JSON; cancellation before transport; disconnected operation guards.
- **2 simulator UI tests:** authenticated fixture telemetry → disconnect → demo; explicit control acquisition → fixture-source check → movement confirmation → simulated motor-service start → stop.
- Production Swift telemetry client/controller against the real gateway fixture: correct decoding, rejected token, corrected credentials, background pause/resume, disconnect cancellation and demo transition.
- Fault harness using the gateway's actual HTTP/state implementation: delayed responses remain stale; a temporarily unavailable gateway is automatically reconnected when it starts.
- Production Swift operations harness against the actual command fixture: 20-operation catalog, exclusive lease, required confirmation, simulated process starts, navigation/joystick conflict rejection, provider-stop dependency rejection, stable command outcomes across heartbeats, ordered shutdown, background lease expiry, and explicit reacquisition after returning to foreground.
- Independent review completed. Fixed network-transit freshness, large-age conversion, misleading demo wording, keyboard dismissal, heartbeat overwriting command results, and cancellation of outstanding command/acquisition requests.
- Visual review of the WHOOP/Tesla-inspired dark redesign. Compact rover hero, readable status and cards, and active jobs/shutdown controls above the command catalog. Fixture labels and accessibility identifiers retained.

The simulator initially stalled during the original telemetry setup; restarting it recovered subsequent runs. Early UI tests also exposed keyboard dismissal and accessibility-selector issues; the passing suites include those fixes.

## Evidence and reproduction

- Unit result: `/tmp/helios-ios-verified/Logs/Test/Test-Helios-2026.09.28_13-48-26--0700.xcresult` (check actual latest bundle in that folder if Xcode regenerates it).
- Final UI result: `/tmp/helios-ios-verified/Logs/Test/Test-HeliosIntegration-2026.09.28_13-49-48--0700.xcresult`.
- Commands and reproduction steps: `README.md`, `OPERATIONS.md`, `Scripts/check-gateway.sh`, `Scripts/check-operations.sh`, and `Tests/run_gateway_faults.py`.
- Captured app screens: `Documentation/fixture-overview.png` and `Documentation/fixture-operations.png`.

## Limits

The tests establish app/API behavior, not real ROS launch success, map-file quality, discovery/QoS, Jetson device permissions, certificate trust on a physical phone, motor stopping behavior, or navigation readiness. The gateway starts only allowlisted owned processes and does not take over laptop-launched stacks. Lease expiry requests orderly shutdown; it is not an emergency stop and cannot guarantee an unresponsive process will stop. Commands default disabled on the real gateway until explicitly enabled by the operator.

Direct velocity commands, navigation goal submission/cancellation, GUI tools, package/device/network administration, calibration, battery, live map rendering and video streaming are not implemented. See `OPERATIONS.md` for the supported 20-operation scope. Hardware testing and physical-phone signing remain deployment work.

---
This file is part of the mirrored [Helios setup bundle](README.md). Paths refer to the original app repository root and local test environment.
