#!/bin/bash
# ==============================================================================
# AI NVR DeepStream & Service Layer 启动脚本 (Jetson Orin 适配版)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/../configs"
SERVICE_DIR="$SCRIPT_DIR/../service"

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
MODE="udp"
NUM_STREAMS=8

while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)       MODE="$2"; shift 2 ;;
        -n|--num)     NUM_STREAMS="$2"; shift 2 ;;
        *)            echo "未知参数: $1"; exit 1 ;;
    esac
done

# ---------------------------------------------------------------------------
# 配置 DeepStream 环境变量
# ---------------------------------------------------------------------------
export DS_ROOT="/opt/nvidia/deepstream/deepstream"
export PATH="$DS_ROOT/bin:$PATH"
export LD_LIBRARY_PATH="$DS_ROOT/lib:/usr/lib/aarch64-linux-gnu/nvidia:$LD_LIBRARY_PATH"
export GST_PLUGIN_PATH="$DS_ROOT/lib/gst-plugins:$GST_PLUGIN_PATH"

# 自动检测 PC IP
RTSP_SERVER_IP=${RTSP_SERVER_IP:-10.42.0.1}

echo "=============================================="
echo "AI NVR DeepStream 启动"
echo "RTSP 服务器: $RTSP_SERVER_IP"
echo "模式: $MODE"
echo "=============================================="

# 1. 生成运行时配置
RUNTIME_CONFIG="$CONFIG_DIR/ds_nvr_config_runtime.txt"
sed "s/RTSP_SERVER_IP/$RTSP_SERVER_IP/g" "$CONFIG_DIR/ds_nvr_config.txt" > "$RUNTIME_CONFIG"

# 2. 启动 Python API
echo "[*] 启动后端服务..."
python3 "$SERVICE_DIR/api.py" --port 8080 --mode "$MODE" &
SERVICE_PID=$!

# 3. 启动桥接 (如果是 udp 模式且是 DS 7.1)
BRIDGE_PID=""
if [ "$MODE" == "udp" ]; then
    echo "[*] 启动 AMQP→UDP 桥接 (针对 DS 7.1)..."
    # 确保 RabbitMQ 运行
    echo "  确保 RabbitMQ 运行..."
    sudo systemctl start rabbitmq-server 2>/dev/null || true
    # 确保 pika 已安装
    python3 -c "import pika" 2>/dev/null || pip3 install pika -q

    python3 "$SERVICE_DIR/amqp_bridge.py" --udp-port 9000 &
    BRIDGE_PID=$!

    # 修改运行时配置以启用 AMQP sink
    # 我们需要在运行时向 ds_nvr_config_runtime.txt 添加 [sink1]
    cat <<EOF >> "$RUNTIME_CONFIG"

[sink1]
enable=1
type=6
msg-conv-config=config_msgconv.txt
msg-broker-proto-lib=/opt/nvidia/deepstream/deepstream/lib/libnvds_amqp_proto.so
msg-broker-conn-str=localhost;5672;guest;guest
topic=nvr-events
sync=0
EOF
fi

trap "kill $SERVICE_PID $BRIDGE_PID 2>/dev/null; rm -f $RUNTIME_CONFIG; exit" INT TERM EXIT

echo "[*] 启动 DeepStream 管线..."
cd "$CONFIG_DIR"
deepstream-app -c ds_nvr_config_runtime.txt