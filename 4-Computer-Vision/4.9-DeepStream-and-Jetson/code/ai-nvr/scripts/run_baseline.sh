#!/bin/bash
# ==============================================================================
# AI NVR 基线实验启动脚本 (Jetson 适配版)
# ==============================================================================
# 用法: ./run_baseline.sh [stream_count]
#   stream_count: 1-8 (默认 4)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASELINE="$SCRIPT_DIR/../baseline/simple_inference.py"

# 参数: 流数量
NUM_STREAMS=${1:-4}

# RTSP 源 IP
if [ -z "$RTSP_SERVER_IP" ]; then
    RTSP_SERVER_IP="10.42.0.1"
fi

# YOLO 模型路径（优先使用环境变量，默认 Jetson 上常见的路径）
YOLO_MODEL=${YOLO_MODEL:-/home/seeed/test/yolo26s.pt}

SOURCES=""
for i in $(seq 1 $NUM_STREAMS); do
    SOURCES="$SOURCES rtsp://${RTSP_SERVER_IP}:8554/cam${i}"
done

echo "=============================================="
echo "AI NVR 基线实验启动"
echo "流数量: $NUM_STREAMS"
echo "源: $SOURCES"
echo "模型: $YOLO_MODEL"
echo "=============================================="

python3 "$BASELINE" --sources $SOURCES \
    --model "$YOLO_MODEL" \
    --conf 0.25 \
    --duration 60