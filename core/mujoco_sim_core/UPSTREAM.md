# Bundled MuJoCo provenance

## Source

- Upstream project: `google-deepmind/mujoco`
- Release: `3.10.0`
- Release URL: `https://github.com/google-deepmind/mujoco/releases/tag/3.10.0`
- Asset: `mujoco-3.10.0-linux-x86_64.tar.gz`
- Published: `2026-06-22T17:04:32Z`
- Upstream archive SHA-256:
  `4b9ba10f1b43fe77768c2354a9789e863972715ceed009e508c55195b35a83c6`

The extracted distribution is stored under
`mujoco/linux/x86/mujoco-3.10.0/`. It is the Linux x86_64 bundle used by the
default build and is not a portable source distribution.

## Verification evidence

The checked-in shared library and license were compared with the corresponding
files streamed directly from the official release asset on 2026-07-22:

| File | SHA-256 |
| --- | --- |
| `lib/libmujoco.so.3.10.0` | `872b2954e9760b1df122e3c807a907ff806e1980099b00e15789ae67e08b75d5` |
| `LICENSE` | `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30` |

Both hashes match the official archive. The upstream license is Apache-2.0 and
is preserved at `mujoco/linux/x86/mujoco-3.10.0/LICENSE`.

## First-party boundary

Files below `mujoco/` are vendored and must remain byte-oriented vendor updates.
Project integration, CMake targets, adapters, tests, and documentation live
outside that tree. The repository does not patch the bundled headers or shared
library to adapt first-party behavior.

Set `-DMUJOCO_SIM_USE_BUNDLED_MUJOCO=OFF` to use a compatible system MuJoCo
CMake package instead of this distribution.
