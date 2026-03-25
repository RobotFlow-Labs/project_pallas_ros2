#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_SIM_ROOT="${PALLAS_SIM_ROOT:-$(cd "${REPO_ROOT}/.." && pwd)/anima-Ros2-Gazebo}"
SIM_ROOT="${DEFAULT_SIM_ROOT}"
PRESET="pallas_core_gazebo.yaml"
WORLD_FILE="terrain.sdf"
IMAGE_TAG="pallas-ros2:gazebo-jazzy"
PALLAS_CONTAINER="pallas_gazebo_live"
SIM_CONTAINER="anima_ros2_gazebo"
USE_GPU=false
START_VIEWER=true
SKIP_SIM_BUILD=false
SKIP_IMAGE_BUILD=false
SKIP_PALLAS_BUILD=false
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

usage() {
  cat <<'EOF'
usage: ./scripts/gazebo_lidar_test.sh [preset.yaml] [options]

Examples:
  ./scripts/gazebo_lidar_test.sh
  ./scripts/gazebo_lidar_test.sh pallas_ct_gazebo.yaml
  ./scripts/gazebo_lidar_test.sh --profile ct --world terrain.sdf
  ./scripts/gazebo_lidar_test.sh --gpu

Options:
  --profile core|ct       Select the shipped Gazebo preset by profile.
  --preset NAME           Explicit preset name. Defaults to pallas_core_gazebo.yaml.
  --world FILE            Gazebo world file. Defaults to terrain.sdf.
  --sim-root PATH         Path to the ANIMA Gazebo repo.
  --gpu                   Use the CUDA-backed Gazebo image path.
  --no-viewer             Do not start the static viewer container.
  --skip-sim-build        Reuse the existing sim image without rebuilding it.
  --skip-image-build      Reuse the existing PALLAS Jazzy image without rebuilding it.
  --skip-pallas-build     Skip colcon build inside the PALLAS container.
  -h, --help              Show this help text.
EOF
}

log() {
  printf '[pallas-gazebo] %s\n' "$*"
}

fail() {
  printf '[pallas-gazebo] ERROR: %s\n' "$*" >&2
  exit 1
}

compose() {
  if docker compose version >/dev/null 2>&1; then
    docker compose "$@"
    return
  fi
  if command -v docker-compose >/dev/null 2>&1; then
    docker-compose "$@"
    return
  fi
  fail "docker compose is not available"
}

wait_for_health() {
  local container_name="$1"
  local timeout_sec="${2:-90}"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local status
    status="$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}{{.State.Status}}{{end}}' "${container_name}" 2>/dev/null || true)"
    if [[ "${status}" == "healthy" || "${status}" == "running" ]]; then
      return 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
  done
  fail "timed out waiting for ${container_name} to become healthy"
}

wait_for_ros_topic() {
  local topic_name="$1"
  local timeout_sec="${2:-45}"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if docker exec "${SIM_CONTAINER}" bash -lc \
      "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -qx '${topic_name}'" \
      >/dev/null 2>&1; then
      return 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
  done
  fail "timed out waiting for ROS topic ${topic_name}"
}

detect_gz_topic() {
  local pattern="$1"
  docker exec "${SIM_CONTAINER}" bash -lc "gz topic -l 2>/dev/null" | awk "${pattern}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      if [[ $# -lt 2 ]]; then
        fail "--profile requires a value"
      fi
      case "$2" in
        core|ct)
          PRESET="pallas_${2}_gazebo.yaml"
          ;;
        *)
          fail "--profile must be core or ct"
          ;;
      esac
      shift 2
      ;;
    --preset)
      if [[ $# -lt 2 ]]; then
        fail "--preset requires a value"
      fi
      PRESET="$2"
      shift 2
      ;;
    --world)
      if [[ $# -lt 2 ]]; then
        fail "--world requires a value"
      fi
      WORLD_FILE="$2"
      shift 2
      ;;
    --sim-root)
      if [[ $# -lt 2 ]]; then
        fail "--sim-root requires a value"
      fi
      SIM_ROOT="$2"
      shift 2
      ;;
    --gpu)
      USE_GPU=true
      shift
      ;;
    --no-viewer)
      START_VIEWER=false
      shift
      ;;
    --skip-sim-build)
      SKIP_SIM_BUILD=true
      shift
      ;;
    --skip-image-build)
      SKIP_IMAGE_BUILD=true
      shift
      ;;
    --skip-pallas-build)
      SKIP_PALLAS_BUILD=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      fail "unknown option: $1"
      ;;
    *)
      PRESET="$1"
      shift
      ;;
  esac
done

[[ -d "${SIM_ROOT}" ]] || fail "sim repo not found: ${SIM_ROOT}"
[[ -f "${REPO_ROOT}/Dockerfile" ]] || fail "PALLAS Dockerfile not found"
[[ -f "${REPO_ROOT}/ros2_ws/src/anima_pallas_ros2/config/${PRESET}" ]] || fail "unknown preset: ${PRESET}"

if [[ "${PRESET}" != pallas_core_gazebo.yaml && "${PRESET}" != pallas_ct_gazebo.yaml ]]; then
  fail "this launcher only supports the Gazebo presets"
fi

if [[ "${SKIP_SIM_BUILD}" != true ]]; then
  log "building Gazebo image (${USE_GPU:+CUDA }path) from ${SIM_ROOT}"
  if [[ "${USE_GPU}" == true ]]; then
    "${SIM_ROOT}/scripts/build.sh" --gpu
  else
    "${SIM_ROOT}/scripts/build.sh"
  fi
fi

pushd "${SIM_ROOT}" >/dev/null
services=(sim)
if [[ "${START_VIEWER}" == true ]]; then
  services+=(viewer)
fi
log "starting Gazebo stack in ${SIM_ROOT} (world=${WORLD_FILE})"
if [[ "${USE_GPU}" == true ]]; then
  WORLD_FILE="${WORLD_FILE}" compose -f docker-compose.yml -f docker-compose.gpu.yml up -d "${services[@]}"
else
  WORLD_FILE="${WORLD_FILE}" compose up -d "${services[@]}"
fi
popd >/dev/null

wait_for_health "${SIM_CONTAINER}" 120

POINTS_GZ_TOPIC="$(detect_gz_topic '/\/scan\/points$/ { print; exit }')"
IMU_GZ_TOPIC="$(detect_gz_topic '/\/sensor\/imu\/imu$/ { print; exit }')"
SCAN_GZ_TOPIC="$(detect_gz_topic '!/\/scan\/points$/ && /\/scan$/ { print; exit }')"

[[ -n "${POINTS_GZ_TOPIC}" ]] || fail "no 3D Gazebo point cloud topic found; use a world with lidar/scan/points such as terrain.sdf"
[[ -n "${IMU_GZ_TOPIC}" ]] || fail "no Gazebo IMU topic found; use a world with an IMU sensor such as terrain.sdf"

log "bridging ${POINTS_GZ_TOPIC} -> /anima/lidar/points"
log "bridging ${IMU_GZ_TOPIC} -> /anima/imu/data"

docker exec "${SIM_CONTAINER}" bash -lc 'pkill -f PALLAS_GAZEBO_BRIDGE || true' >/dev/null 2>&1 || true
docker exec -d \
  -e POINTS_GZ_TOPIC="${POINTS_GZ_TOPIC}" \
  -e IMU_GZ_TOPIC="${IMU_GZ_TOPIC}" \
  -e SCAN_GZ_TOPIC="${SCAN_GZ_TOPIC}" \
  "${SIM_CONTAINER}" \
  bash -lc '
    set -euo pipefail
    source /opt/ros/jazzy/setup.bash
    bridge_args=(
      "${POINTS_GZ_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
      "${IMU_GZ_TOPIC}@sensor_msgs/msg/Imu[gz.msgs.IMU"
    )
    remap_args=(
      -r "${POINTS_GZ_TOPIC}:=/anima/lidar/points"
      -r "${IMU_GZ_TOPIC}:=/anima/imu/data"
    )
    if [[ -n "${SCAN_GZ_TOPIC}" ]]; then
      bridge_args+=("${SCAN_GZ_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
      remap_args+=(-r "${SCAN_GZ_TOPIC}:=/anima/lidar/scan")
    fi
    exec -a PALLAS_GAZEBO_BRIDGE \
      ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" --ros-args "${remap_args[@]}" \
      >>/tmp/pallas_gazebo_bridge.log 2>&1
  '

wait_for_ros_topic "/anima/lidar/points" 45
wait_for_ros_topic "/anima/imu/data" 45

if [[ "${SKIP_IMAGE_BUILD}" != true ]]; then
  log "building PALLAS Jazzy image ${IMAGE_TAG}"
  docker build --build-arg ROS_DISTRO=jazzy -t "${IMAGE_TAG}" "${REPO_ROOT}"
fi

docker rm -f "${PALLAS_CONTAINER}" >/dev/null 2>&1 || true

log "viewer: http://localhost:8080"
log "running PALLAS in Docker on anima_ros2_sim_net with preset ${PRESET}"

RUNTIME_CMD=$(
  cat <<EOF
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
$(if [[ "${SKIP_PALLAS_BUILD}" != true ]]; then printf 'uv run pallas-dev build\nif [[ -f /workspace/install/setup.bash ]]; then set +u\nsource /workspace/install/setup.bash\nset -u\nfi\n'; else printf 'if [[ -f /workspace/install/setup.bash ]]; then set +u\nsource /workspace/install/setup.bash\nset -u\nfi\n'; fi)
uv run pallas-dev ros-check ${PRESET}
exec uv run pallas-dev launch-live ${PRESET} --skip-ros-check
EOF
)

docker run --rm -it \
  --name "${PALLAS_CONTAINER}" \
  --network anima_ros2_sim_net \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}" \
  "${IMAGE_TAG}" \
  bash -lc "${RUNTIME_CMD}"
