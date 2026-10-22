#!/bin/bash
# Download the hash-verified Voron V0.2 CAD and generate the printer model's visual
# meshes into arctos_gazebo/models/voron_v02/meshes. Leaves model.sdf untouched.
# Run before `colcon build`. See docs/VORON_SIM_MODEL.md.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAD_DIR="${VORON_CAD_DIR:-$HOME/voron-inspection}"
CADGEN=(uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26)

if ! command -v uv >/dev/null 2>&1; then
    echo "uv is required: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
fi

# Keep a sourced ROS environment's Python paths out of the isolated CAD runtime.
unset PYTHONPATH

echo "Fetching and verifying Voron CAD in $CAD_DIR"
python3 "$REPO/scripts/inspect_voron_cad.py" --fetch-only --directory "$CAD_DIR"

CADGEN_TELEMETRY=0 "${CADGEN[@]}" cadgen telemetry off >/dev/null
echo "Generating meshes (first run downloads the CAD runtime and imports a 179 MB STEP)"
CADGEN_TELEMETRY=0 "${CADGEN[@]}" python "$REPO/scripts/build_voron_sdf.py" \
    --directory "$CAD_DIR" --meshes-only "$@"

ls -lh "$REPO/arctos_gazebo/models/voron_v02/meshes"
