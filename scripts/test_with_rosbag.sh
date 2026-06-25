#!/usr/bin/env bash
# AutoNav-SLAM rosbag entegrasyon testi.
# MulRan KAIST01 (veya başka bir sequence) verisiyle SLAM node'unu test eder.
#
# Kullanım:
#   bash scripts/test_with_rosbag.sh [SEQUENCE]   # default: KAIST01
#
# Docker içinde: CMD olarak çağrılır.
# Doğrudan: source /opt/ros/humble/setup.bash && source install/setup.bash önce.

set -e

SEQUENCE="${1:-KAIST01}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_DIR="${REPO_ROOT}/data/mulran"
BAG_DIR="${DATA_DIR}/${SEQUENCE}/rosbag2"
RESULT_FILE="${REPO_ROOT}/data/test_result_${SEQUENCE}.txt"

echo "======================================="
echo " AutoNav-SLAM Rosbag Entegrasyon Testi"
echo " Sequence : ${SEQUENCE}"
echo " Bag dir  : ${BAG_DIR}"
echo "======================================="

# ── 1. Rosbag yoksa indir ve dönüştür ─────────────────────────────
if [ ! -d "${BAG_DIR}" ]; then
    echo "[1/4] MulRan verisi indiriliyor..."
    mkdir -p "${DATA_DIR}"
    python3 "${SCRIPT_DIR}/download_mulran.py" \
        --sequence "${SEQUENCE}" \
        --out "${DATA_DIR}"

    echo "[1/4] ROS2 bag formatına dönüştürülüyor..."
    if command -v mulran2bag &>/dev/null; then
        mulran2bag \
            --input "${DATA_DIR}/${SEQUENCE}" \
            --output "${BAG_DIR}" \
            --lidar-topic /lidar/points \
            --imu-topic   /imu/data
    else
        echo "UYARI: mulran2bag bulunamadı — 'pip install mulran2bag' çalıştır"
        exit 1
    fi
else
    echo "[1/4] Mevcut bag kullanılıyor: ${BAG_DIR}"
fi

# ── 2. SLAM node başlat ────────────────────────────────────────────
echo "[2/4] SLAM node başlatılıyor..."
ros2 launch autonav_slam slam.launch.py \
    voxel_size:=0.3 \
    use_rviz:=false &
SLAM_PID=$!
echo "      PID: ${SLAM_PID}"

sleep 4  # node başlamasını bekle

# ── 3. Rosbag oynat ───────────────────────────────────────────────
echo "[3/4] Bag oynatılıyor (MulRan remap'leri ile)..."
ros2 bag play "${BAG_DIR}" \
    --rate 1.0 \
    --remap /os_cloud_node/points:=/lidar/points \
    --remap /imu/data:=/imu/data &
BAG_PID=$!

echo "      20 saniye bekleniyor..."
sleep 20

# ── 4. Topic kontrol ──────────────────────────────────────────────
echo "[4/4] Topic çıktıları kontrol ediliyor..."
PASS=0
FAIL=0

check_topic() {
    local topic="$1"
    local result
    result=$(ros2 topic echo "${topic}" --once --no-daemon 2>&1) && {
        echo "OK : ${topic}"
        PASS=$((PASS + 1))
    } || {
        echo "FAIL: ${topic}"
        FAIL=$((FAIL + 1))
    }
}

check_topic /slam/odometry
check_topic /slam/pose
check_topic /slam/map

echo ""
echo "TF ağacı:"
ros2 run tf2_tools view_frames --spin-time 2 2>/dev/null \
    && echo "TF frames.pdf oluşturuldu" || echo "TF kontrolü atlandı"

# ── Temizle ───────────────────────────────────────────────────────
kill "${BAG_PID}"  2>/dev/null || true
kill "${SLAM_PID}" 2>/dev/null || true

# ── Sonuç ─────────────────────────────────────────────────────────
echo ""
echo "======================================="
echo " SONUC: ${PASS} BASARILI / ${FAIL} BASARISIZ"
echo "======================================="
{
    echo "sequence=${SEQUENCE}"
    echo "pass=${PASS}"
    echo "fail=${FAIL}"
    echo "ts=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "${RESULT_FILE}"
echo "Sonuc yazildi: ${RESULT_FILE}"

[ "${FAIL}" -eq 0 ]  # exit 0 = success, exit 1 = failure
