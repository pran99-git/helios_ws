# Real robot and developer modes

Every build starts in real robot mode. Overview shows authenticated ROS gateway readings or “No live readings.” Map shows “Live map unavailable”; the API has no captured-map endpoint. No sample values fill missing sensors. The decorative rover schematic is not sensor data.

## Distribution policy

| Build configuration | Developer tools |
| --- | --- |
| Debug | Available after an explicit foreground-session opt-in |
| Release (ordinary production) | Unavailable, regardless of receipt, settings, launch arguments, or StoreKit evidence |
| TestFlight (`HeliosTestFlight` scheme) | Available only after StoreKit returns a verified sandbox AppTransaction with the app's matching bundle identifier, then explicit opt-in |

The TestFlight build uses `TESTFLIGHT_DEVELOPER_TOOLS`, not `DEBUG`. Ordinary Release has neither condition and also rejects explicit fixture-mode client construction at the transport initializer. No receipt filename, environment variable, preferences key, URL scheme, hidden gesture, or remote override grants access. Missing, unverified, production, Xcode, unknown-environment, mismatched-identity, cancelled, or failed verification stays closed. Eligibility is checked on launch; relaunch to retry unavailable verification. Opt-in is never persisted.

Apple's [AppTransaction.shared](https://developer.apple.com/documentation/storekit/apptransaction/shared) supplies verified app-transaction evidence. Its [environment](https://developer.apple.com/documentation/storekit/apptransaction/environment) identifies sandbox versus production, **not a unique TestFlight installation channel**. A sandbox receipt filename alone is not sufficient and is not used.

For beta distribution, archive the **HeliosTestFlight** scheme and choose **TestFlight Internal Only** in Xcode's distribution workflow. Apple's [distribution guide](https://developer.apple.com/documentation/Xcode/distributing-your-app-for-beta-testing-and-releases) says this prevents submission of that build to the App Store or external testers. This distribution restriction and verified sandbox check work together; the code does not claim cryptographic proof of the TestFlight channel. For ordinary production, archive **Helios**, whose Archive configuration remains **Release**. Do not repurpose the internal beta archive as the production artifact.

## Using developer tools

In an eligible build, open **Connect → Enable developer mode → Enable simulation**. A persistent “DEVELOPER MODE • Simulation only” banner spans all tabs. Overview initially shows the local sample and Map shows the labeled synthetic room. **Use local sample** disconnects fixture polling and returns to the offline preview. To exercise network fixtures, connect to the synthetic fixture gateway while developer mode is active; fixture telemetry and operations have additional simulated labels.

Developer mode accepts only `source: fixture` telemetry/catalogs and simulated jobs. It rejects the real `ros2` gateway. Real robot mode accepts only `source: ros2` telemetry/catalogs and non-simulated jobs; missing/unknown sources are rejected. HTTPS is required for real connections. HTTP loopback is restricted to explicit fixture mode; a phone's localhost is the phone itself. Off-device fixtures still require trusted HTTPS.

Switching modes disconnects telemetry and operations, cancels outstanding requests, clears cached data/jobs/catalog and local control, and resets local playback. **Return to real robot mode** requires a new connection. Leaving the active scene (including backgrounding or locking) exits developer mode; returning never restores opt-in or reacquires control. Ordinary real-mode telemetry resumes as before, with explicit control reacquisition required.

Disconnecting is not a physical stop. A command already accepted may still run until the gateway's lease expires; check the robot and stop managed processes before changing modes. Existing confirmation, ordered shutdown, dependency checks, and uncertain-outcome messages remain in place.

## Protocol limits and command checks

Each command mutation first fetches and validates the catalog's source and job flags using the same cancellable request scope, then sends the mutation once. This catches accidental endpoint/source changes before lease acquisition or a command. Rejected responses never enter session state. A lost mutation response still has an uncertain outcome and is never automatically retried.

These checks rely on the authenticated gateway's truthful metadata. They cannot detect ROS bag replay, a dishonest server claiming `ros2`, or eliminate a server change between preflight and mutation. This app-side separation is not physical-hardware attestation. Keep production deployment pointed at the trusted ROS gateway.

## Verification and release checklist

The separate validation record reports actual local checks. Before distributing:

1. Install a signed Internal Only TestFlight archive on a physical phone; verify tools appear only after verified evidence and remain off until confirmation.
2. Confirm ordinary App Store/production Release has no developer tools, even with prior beta preferences or restored app data.
3. Check fresh-install/offline/missing evidence stays closed; verified cached evidence may work offline. Relaunch after connectivity recovers.
4. Verify opt-in, fixture labels, return to robot mode, background/lock, app termination, update/reinstall, and no automatic control reacquisition.
5. Complete separate supervised phone/Jetson TLS, ROS, motor-stop, and hardware commissioning checks.

No signed TestFlight, App Store installation, physical phone, or rover validation has been performed by these local tests.
