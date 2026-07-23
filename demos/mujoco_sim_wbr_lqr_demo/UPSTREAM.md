# Upstream provenance

## Sources

- Repository: `https://github.com/CosmosMount/wbr_mujoco.git`
- Default branch: `new`
- Commit: `1d895574d91e3b14d13696676d9f91b2b4403ee4`
- Commit date: `2026-07-17T23:21:23+08:00`
- Subject: `update : README`

The simulator/input contract was reviewed from:

- Repository: `https://github.com/CosmosMount/mujoco_interface.git`
- Default branch: `architecture`
- Commit: `a2501454b09038469137deccb1d8c6ee460d1c69`
- Commit date: `2026-07-14T23:05:16+08:00`
- Subject: `feat: reset after controller off`

## License status

Neither pinned tree contains a `LICENSE`, `COPYING`, or `NOTICE` file, and their
English and Chinese README files state no license. This was verified from fresh
clones on 2026-07-23. Consequently this package declares `NOASSERTION`; it does
not claim that the imported files are covered by an open-source license.

There was no upstream license or notice file to copy into the vendored tree.
Any future vendor update must repeat the license audit and preserve any license
or notice that upstream has added before importing new bytes.

## Imported runtime files

The following controller headers are byte-identical to the fixed commit:

| Upstream path | SHA-256 |
| --- | --- |
| `controller/include/control/balance.hpp` | `cb32fea1f8da52011d68d5bcd93809727d69196e60cb3aaa90f1100f6e4dbc29` |
| `controller/include/control/chassis_fsm.hpp` | `12cdf51e682b7015c75773510b85041e2ef2066c593c878d933adf2fa8b595b9` |
| `controller/include/control/config.hpp` | `c1fc2406f2d1dab6ab34048a236ad901edca0510a0d30c8e8f99e92a9bd8ec35` |
| `controller/include/control/imu_fusion.hpp` | `6a32572535eae8a7e2e06c2f244861da606569e0e0cc0051af887de23fc7c4b7` |
| `controller/include/control/leg.hpp` | `c4176fbf3ae0911466961fb1becd605c528245efdba939ff1c17c1bef2b87474` |
| `controller/include/control/lqr_coeffs.hpp` | `86b1078956b6caa02625170205d9d96ab10dc9f51f38a9553f1dc9ab4019c323` |
| `controller/include/control/math.hpp` | `4ed0fc015e585a2f75cbf863c424b79a2fd53cff9d857f189a66fafe639fa663a6` |
| `controller/include/control/msgs.hpp` | `113974ea09117dbd501e27ddc15a092b194103accb46f4a057c4fa002e0ae147` |

The two MJCF files and all 15 files under `mjcf/meshes/` were imported. Every
mesh and `scene.xml` is byte-identical to upstream. `mjmodel.xml` has one
intentional local change:

```xml
<option timestep="0.001"/>
```

- Upstream `mjmodel.xml` SHA-256:
  `5d67cef7f7712efd0d04f43855dcc1768bfc0a51f00f2f2d2ced88f174cb765d`
- Local `mjmodel.xml` SHA-256:
  `fcb141b573f759e9765a939841875cc7c23b6cb049ad0c970679f5ec7aa6aac3`
- `scene.xml` SHA-256:
  `0baa383496d9bb9a2f8da6aff787eb4ac30377d3e4827f881782741d18aa027e`

Re-audit imported bytes with:

```bash
sha256sum \
  third_party/wbr_mujoco/controller/include/control/*.hpp \
  mjcf/scene.xml mjcf/meshes/*.STL
```

## Explicit exclusions

No files were imported from these `wbr_mujoco` areas:

- `common/`
- `controller/include/controller/` or `controller/src/`
- `tools/lqr/`
- `tools/fix_stl_for_mujoco.py`
- threaded controller application, CLI, and YAML loader

Only the runtime control equations/data and the model assets are vendored.

No source file is copied from `mujoco_interface`. Its
`protocol::input_message` snapshot fields and GUI publication behavior define
the input semantics used by this demo's deterministic `InputScript`. The local
Core, Protocol V4, Server, and Adapter SDK remain the simulation and transport
implementation because repository architecture forbids replacing those
production boundaries with a demo dependency. The interface-only `r` reset key
is deliberately excluded because model reset is a Server lifecycle operation,
not a Controller command.
