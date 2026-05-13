#!/bin/bash
# ==============================================================================
# Jetson AI NVR 环境自动配置脚本 (一键版)
# ==============================================================================
# 用法: ./setup_jetson.sh
# 在 Jetson 设备上运行此脚本，自动完成所有环境配置
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/.."
DS_ROOT="/opt/nvidia/deepstream/deepstream"
DS_LIB="$DS_ROOT/lib"
DS_BIN="$DS_ROOT/bin"
DS_GST="$DS_ROOT/lib/gst-plugins"
DS_DEB="deepstream-7.1"

echo "=============================================="
echo "Jetson AI NVR 环境自动配置"
echo "=============================================="

# ---------------------------------------------------------------------------
# 辅助函数
# ---------------------------------------------------------------------------
run_sudo() {
    # 用当前用户的 sudo 权限执行命令，不硬编码密码
    # 会弹出密码提示让用户交互输入
    sudo "$@"
}

# ---------------------------------------------------------------------------
# 1. 安装系统依赖 (libgstrtspserver + libyaml-cpp-dev + ffmpeg)
# ---------------------------------------------------------------------------
echo ""
echo "[1/6] 安装系统依赖..."

PACKAGES_TO_INSTALL=""

if ! dpkg -s libgstrtspserver-1.0-0 >/dev/null 2>&1; then
    PACKAGES_TO_INSTALL="$PACKAGES_TO_INSTALL libgstrtspserver-1.0-0 libgstrtspserver-1.0-dev"
fi

if ! dpkg -s libyaml-cpp-dev >/dev/null 2>&1; then
    PACKAGES_TO_INSTALL="$PACKAGES_TO_INSTALL libyaml-cpp-dev"
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
    PACKAGES_TO_INSTALL="$PACKAGES_TO_INSTALL ffmpeg"
fi

if [ -n "$PACKAGES_TO_INSTALL" ]; then
    echo "  安装$PACKAGES_TO_INSTALL ..."
    run_sudo apt-get update -qq
    run_sudo apt-get install -y $PACKAGES_TO_INSTALL
    echo "  系统依赖安装完成"
else
    echo "  系统依赖已就绪"
fi

# ---------------------------------------------------------------------------
# 2. 安装 DeepStream SDK
# ---------------------------------------------------------------------------
echo ""
echo "[2/6] 安装 DeepStream SDK..."

if [ -x "$DS_BIN/deepstream-app" ]; then
    DS_VER=$("$DS_BIN/deepstream-app" --version-all 2>&1 | grep "deepstream-app version" | awk '{print $NF}')
    echo "  DeepStream $DS_VER 已安装，跳过"
else
    if apt-cache show "$DS_DEB" >/dev/null 2>&1; then
        echo "  通过 apt 安装 $DS_DEB（约 600MB，请耐心等待）..."
        run_sudo apt-get install -y "$DS_DEB"
        echo "  DeepStream 安装完成"
    else
        echo "  ❌ apt 仓库中未找到 $DS_DEB"
        echo "  请手动安装 DeepStream: https://docs.nvidia.com/deepstream/latest/deployment-guide/platform-guide.html"
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# 3. 配置环境变量 (写入 ~/.bashrc，幂等)
# ---------------------------------------------------------------------------
echo ""
echo "[3/6] 配置环境变量..."

ENV_BLOCK='# --- DeepStream AI NVR Environment ---
export PATH="/opt/nvidia/deepstream/deepstream/bin:$PATH"
export LD_LIBRARY_PATH="/opt/nvidia/deepstream/deepstream/lib:/usr/lib/aarch64-linux-gnu/nvidia:$LD_LIBRARY_PATH"
export GST_PLUGIN_PATH="/opt/nvidia/deepstream/deepstream/lib/gst-plugins:$GST_PLUGIN_PATH"
# --- End DeepStream ---'

if ! grep -q "DeepStream AI NVR Environment" ~/.bashrc 2>/dev/null; then
    echo "" >> ~/.bashrc
    echo "$ENV_BLOCK" >> ~/.bashrc
    echo "  环境变量已写入 ~/.bashrc"
else
    echo "  环境变量已存在于 ~/.bashrc"
fi

# 立即生效（当前 shell）
export PATH="$DS_BIN:$PATH"
export LD_LIBRARY_PATH="$DS_LIB:/usr/lib/aarch64-linux-gnu/nvidia:$LD_LIBRARY_PATH"
export GST_PLUGIN_PATH="$DS_GST:$GST_PLUGIN_PATH"

# ---------------------------------------------------------------------------
# 4. 验证 DeepStream 安装
# ---------------------------------------------------------------------------
echo ""
echo "[4/6] 验证 DeepStream..."

if deepstream-app --version-all 2>&1 | grep -q "deepstream-app version"; then
    DS_VER=$(deepstream-app --version-all 2>&1 | grep "deepstream-app version" | awk '{print $NF}')
    echo "  DeepStream $DS_VER 可运行"
else
    echo "  ❌ deepstream-app 无法运行，请检查安装"
    exit 1
fi

# ---------------------------------------------------------------------------
# 5. 生成 TensorRT 引擎 (如果不存在)
# ---------------------------------------------------------------------------
echo ""
echo "[5/6] 检查 TensorRT 推理引擎..."

MODEL_DIR="$DS_ROOT/samples/models/Primary_Detector"
ONNX_FILE="$MODEL_DIR/resnet18_trafficcamnet_pruned.onnx"
ENGINE_FILE="$MODEL_DIR/resnet18_trafficcamnet_pruned.onnx_b8_gpu0_int8.engine"
CAL_FILE="$MODEL_DIR/cal_trt.bin"

if [ -f "$ENGINE_FILE" ]; then
    echo "  推理引擎已存在: $ENGINE_FILE"
else
    echo "  生成 TensorRT 引擎（首次运行需要较长时间）..."
    if command -v trtexec >/dev/null 2>&1; then
        trtexec \
            --onnx="$ONNX_FILE" \
            --int8 \
            --calib="$CAL_FILE" \
            --saveEngine="$ENGINE_FILE" \
            --batch=8 \
            --devices 0 2>&1 | tail -5
        if [ -f "$ENGINE_FILE" ]; then
            echo "  推理引擎生成成功"
        else
            echo "  trtexec 生成失败，将在 DeepStream 首次运行时自动生成"
        fi
    else
        echo "  trtexec 不可用，将在 DeepStream 首次运行时自动生成引擎"
    fi
fi

# ---------------------------------------------------------------------------
# 6. 安装 Python 依赖
# ---------------------------------------------------------------------------
echo ""
echo "[6/6] 安装 Python 依赖..."

PYTHON_DEPS=""
python3 -c "import aiohttp" 2>/dev/null || PYTHON_DEPS="$PYTHON_DEPS aiohttp"
python3 -c "import pika" 2>/dev/null || PYTHON_DEPS="$PYTHON_DEPS pika"
python3 -c "import ultralytics" 2>/dev/null || PYTHON_DEPS="$PYTHON_DEPS ultralytics"

if [ -n "$PYTHON_DEPS" ]; then
    echo "  安装$PYTHON_DEPS ..."
    pip3 install --break-system-packages $PYTHON_DEPS -q 2>/dev/null \
        || pip3 install $PYTHON_DEPS -q
    echo "  Python 依赖安装完成"
else
    echo "  Python 依赖已就绪"
fi

# ---------------------------------------------------------------------------
# 安装 RabbitMQ (AMQP 模式需要)
# ---------------------------------------------------------------------------
echo ""
echo "  [可选] 安装 RabbitMQ (AMQP 元数据桥接需要)..."
if ! dpkg -s rabbitmq-server >/dev/null 2>&1; then
    read -r -p "  是否安装 RabbitMQ？[y/N] " REPLY
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        run_sudo apt-get install -y rabbitmq-server
        run_sudo systemctl enable rabbitmq-server
        run_sudo systemctl start rabbitmq-server
        echo "  RabbitMQ 安装并启动完成"
    else
        echo "  跳过 RabbitMQ（可使用 --mode file 代替）"
    fi
else
    echo "  RabbitMQ 已安装"
fi

# ---------------------------------------------------------------------------
# 验证项目配置文件
# ---------------------------------------------------------------------------
echo ""
echo "=============================================="
echo "环境配置完成！"
echo ""
echo "下一步："
echo "  1. 在 PC 上运行: python3 sim_rtsp_stream.py"
echo "  2. 在 Jetson 上运行: cd scripts && RTSP_SERVER_IP=<PC_IP> ./run_nvr.sh"
echo ""
echo "注意: 如果刚配置了环境变量，请先运行: source ~/.bashrc"
echo "=============================================="