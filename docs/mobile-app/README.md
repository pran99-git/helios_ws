# Helios: robot and iPhone setup

Start with [Pranav’s complete setup checklist](PRANAV_SETUP.md). It takes you from reviewing and merging the gateway pull request through robot setup, secure phone access, installing the app, and supervised acceptance checks.

## Complete guide bundle

- [Pranav’s end-to-end handoff](PRANAV_SETUP.md) — deployment order, required values, commissioning, and troubleshooting.
- [Gateway setup](GATEWAY_SETUP.md) — ROS telemetry contract, build, authentication, and HTTPS.
- [Gateway operations](GATEWAY_OPERATIONS.md) — supported commands, parameters, process ownership, lease handling, and proxy routes.
- [iOS app setup](IOS_APP_SETUP.md) — Xcode, phone installation, connection, and development tests.
- [iOS app operations](IOS_APP_OPERATIONS.md) — control-session workflow, jobs, and uncertain outcomes.
- [Original integration audit](INTEGRATION_AUDIT.md) — pinned source review and ROS assumptions; historical findings are not current deployment status.
- [App validation record](VALIDATION.md) — simulated checks and the physical-hardware validation still required.

The complete bundle is mirrored in the [private iOS repository](https://github.com/tanmayg1502/helios-ios/tree/main/Documentation/Setup) and the [gateway pull request branch](https://github.com/tanmayg1502/helios_ws/tree/codex/mobile-telemetry-gateway/docs/mobile-app). Pranav can read all guides from the robot repository without access to the private app repository. Building the app source still requires access to that private repository.

These guides document the current implemented app and gateway. Real robot mode is the default; ordinary Release excludes access to developer tools. See [developer-mode distribution and validation requirements](DEVELOPER_MODE.md). No physical rover or phone deployment has been validated. Source-code links lead to the relevant repository; commands must run from the repository root specified by each guide, not this documentation folder.

When behavior changes, update the canonical guide and both copies of this bundle together. Never place deployment tokens or private credentials in these files.
