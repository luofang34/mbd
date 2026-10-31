# mbd

Model-based design foundations shared by simulators and design tools, with no
dependency on either.

| Package | Role |
|---|---|
| `mbd-core` | `no_std` contracts: live component tree, mounting-site physics, layered ports, pluggable aerodynamics |
| `syspack` | Universal system packages: the `syspack.json` schema, SHA-256 asset verification, and a fail-closed loader for vehicles and stationary systems alike |

A system package is a folder holding a `syspack.json` manifest and the asset
files it lists with their SHA-256 and size. Changing an asset means updating
both in the manifest; a package that does not verify will not load. The manifest
itself is the trust root, so a package received over a network needs its own
signature or pinned digest.

`kind` says what the system is, from `FixedWing` to `GroundStation` and
`TestBench`. `dynamics` says how it moves, and `None` marks a system that does
not. `controls` lists the operator controls it has, and is optional.

## Check

```sh
./scripts/ci.sh
```

CI needs the `wasm32-unknown-unknown` target. `mbd-core` builds without `std`.
