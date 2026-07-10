# Upstream provenance

## Source

- Repository: `https://github.com/CosmosMount/wbr_mujoco.git`
- Local source checkout: `/tmp/wbr_mujoco`
- Commit: `6a8b37517803db616d4fb19dcdd5ab448bff7847`
- Commit date: `2026-07-09T23:26:40+08:00`
- Subject: `fix : wheel & forces`

License handling was explicitly outside the scope of this import. No license
files or license conclusions are added by this package.

## Imported runtime files

The following controller headers are byte-identical to the fixed commit:

| Upstream path | SHA-256 |
| --- | --- |
| `controller/include/control/balance.hpp` | `c2d97c063dce5ef557928646d1fd61dbba0581e446b158b5658ffdd966d41da6` |
| `controller/include/control/chassis_fsm.hpp` | `38f85a7d8fe24e32eef622a583e22ffd8d1389074d1b37f54bc44a3b749f5cf4` |
| `controller/include/control/config.hpp` | `c22f5977cef5ec6e5901fa239b85176498b67f435a275a8d4d3811ef7576ac76` |
| `controller/include/control/imu_fusion.hpp` | `6a32572535eae8a7e2e06c2f244861da606569e0e0cc0051af887de23fc7c4b7` |
| `controller/include/control/leg.hpp` | `9e1b798e914a5be43c4c69e053868e9da3435414dac9f066f1b1891064cca864` |
| `controller/include/control/lqr_coeffs.hpp` | `a89691533b458e08f761b9a3cb8b8d54ce865e965efab9975f2ff7ef750c95d5` |
| `controller/include/control/math.hpp` | `4ed0fc015e585a2f75cbf863c424b79a2fd53cff9d857f189a66fafe639fa663a6` |
| `controller/include/control/msgs.hpp` | `7c3c56db807b110ad8e8d0d32fdf63149eec9b972471ddf0d15aff39a34c63a6` |

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

No files were imported from these upstream areas:

- `bridge/`
- `common/include/robot_ipc/`
- `common/src/msg.cpp`
- `sim/` or viewer/input-hub integration
- `tools/lqr/`
- `scripts/fix_stl_for_mujoco.py`
- threaded controller application, CLI, and YAML loader

Only the runtime control equations/data and the model assets are vendored.
