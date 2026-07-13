# Phase-one V2 cutover

This release is a breaking, atomic cutover. Rebuild and redeploy the Server, protocol library, Core, Adapter SDK, Sample Adapter, WBR demo, Sensor demo, and status tool together.

Only V2.0 endpoints exist. Registration uses the unified Adapter lifecycle and `adapter_id`; Tick contains a layout hash while the complete layout is read from registration. Status consumers read only `ServerStatus.adapters`.

The obsolete startup timeout setting and command-line option are removed. Activation uses `activation_timeout_ms`; runtime Barrier and heartbeat deadlines use `tick_timeout_ms`. Strict YAML rejects the removed field and the CLI rejects the removed option.

There is no V1 endpoint, proxy, conversion library, alias, wrapper, mixed-version deployment procedure, or compatibility mode.
