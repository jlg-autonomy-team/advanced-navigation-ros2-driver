# Contributor guide: `ros2-driver`

## Workspace and overlay

This is a git submodule in the ROS 2 Jazzy workspace `/Data/ADV/certus_eval`. Work from the workspace root: source `/opt/ros/jazzy/setup.bash`, build selected packages with `colcon`, then source `install/setup.bash`. Overlay sibling packages only when integrating; do not build from this submodule directory as though it were the workspace root.

## Ownership boundaries

- `adnav_driver`: C++ device node, ANPP decoding, ROS publication/services, transport selection, and logging.
- `adnav_interfaces`: ROS interface package. Its definitions are sourced from the vendored `an-ros-common` submodule; treat that checkout as vendor-owned and leave it untouched unless explicitly assigned.
- `adnav_launch`: launch entrypoints and transport-specific YAML profiles.
- `atlas_ins_odom_bridge` owns derived covariance-aware odometry; `atlas_ntrip_client` is the separate higher-level NTRIP supervisor.

## Authoritative sources

For behavior/API truth, inspect `adnav_driver/src/adnav_driver.cpp`, `adnav_driver/include/adnav_driver/`, `adnav_driver/test/`, `adnav_interfaces/CMakeLists.txt`, the referenced `an-ros-common` message/service definitions, and `adnav_launch/launch/` plus `adnav_launch/config/`. Package dependencies are authoritative in each package's `package.xml`. Existing prose is secondary to these files.

## Focused commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select adnav_interfaces adnav_driver adnav_launch
colcon test --packages-select adnav_interfaces adnav_driver adnav_launch
colcon test-result --verbose
```

For C++ style checks, build/test with `BUILD_TESTING=ON`; `adnav_driver` includes `ament_lint_common`, and its current CMake test target covers the packet contract. To regenerate the compilation database for Serena/clangd from the workspace root:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers compile_commands+
```

## Safety and correctness invariants

- Preserve ANPP packet ID to decoder/update-source mapping; do not silently merge packet sources or label unsupported data as decoded.
- Timestamps from packet 20 are used only when UTC is initialized and valid; otherwise use receipt time. Keep published stamps monotonic.
- ROS IMU covariance fields that lack sensor-reported uncertainty must remain marked unknown, not zero-variance/perfect certainty.
- Validate packet period arrays as alternating packet ID/period pairs and maintain `publish_us >= read_us`.
- Installation alignment changes device state. Keep calibration deliberate and do not trigger it during ordinary startup; service callers must ensure stationary, level conditions before persisting alignment.
- Never put NTRIP credentials in checked-in launch YAML or logs intended for source control.

## Generated and sensitive files

Do not commit workspace `build/`, `install/`, or `log/` trees, generated compilation databases, device capture/log files (`Log_*.anpp`, `*.rtcm`), or credentials. Do not edit files inside `an-ros-common` for ordinary driver tasks.

## Serena navigation

Activate the parent workspace project `/Data/ADV/certus_eval`; submodules are navigated through that project, not activated as separate Serena roots. The workspace `.serena/project.yml` uses the C++ language server `clangd`, reading `build/compile_commands.json` produced with the compilation-database build command above. Verify initialization with one `get_symbols_overview` call on `src/ros2-driver/adnav_driver/src/adnav_driver.cpp`. If the language server is uninitialized, restart the Serena MCP server and re-activate `/Data/ADV/certus_eval`.
