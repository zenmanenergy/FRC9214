# arena_description

## CMakeLists.txt
Registers arena_description as an ament_cmake package.

## package.xml

Installs urdf/ into share/arena_description/urdf, which is what get_package_share_directory("arena_description") relies on.

## Validation

`colcon list` now shows arena_description.
`colcon build --packages-select arena_description` passes.
`colcon build --packages-up-to arena_bringup` passes.

## Build just this package

```bash
cd team9214_ws
colcon build --packages-up-to arena_bringup
source install/setup.bash
```

