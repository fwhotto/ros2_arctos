#!/usr/bin/env bash
# Run with bash. Re-exec into a clean environment before sourcing Jazzy.
set -eo pipefail

if [[ "${ARCTOS_CLEAN_ENV:-}" != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" \
    PATH=/usr/bin:/bin LANG=C.UTF-8 ARCTOS_CLEAN_ENV=1 \
    bash "$(realpath "$0")" "$@"
fi

source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo "Gate A requires Ubuntu 24.04." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
set -u

root="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p /tmp/opencode
output="${1:-$(mktemp -d /tmp/opencode/arctos-jazzy.XXXXXX)}"
mkdir -p "$output"
output="$(realpath "$output")"
if [[ -e "$output/build" || -e "$output/install" || -e "$output/log" ]]; then
  echo "Use a fresh output directory: $output" >&2
  exit 1
fi
echo "Validation output: $output"
cd "$root"

packages=(arctos_description arctos_control arctos_moveit_config arctos_gazebo)
rosdep check --from-paths "${packages[@]}" --ignore-src --rosdistro jazzy
colcon list --base-paths "${packages[@]}" --names-only > "$output/packages.txt"
dpkg-query -W 'ros-jazzy-*' > "$output/ros-packages.txt"
gz sim --versions > "$output/gazebo-version.txt"
grep -Eq '^8\.' "$output/gazebo-version.txt"

colcon --log-base "$output/log" build --base-paths "${packages[@]}" \
  --build-base "$output/build" --install-base "$output/install" \
  --cmake-args -DBUILD_TESTING=ON

# ROS setup scripts are not guaranteed to be nounset-compatible.
set +u
source "$output/install/setup.bash"
set -u
colcon --log-base "$output/log" test --base-paths "${packages[@]}" \
  --build-base "$output/build" --install-base "$output/install" \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base "$output/build" --verbose
echo "Jazzy dependency/build/model validation passed: $output"
