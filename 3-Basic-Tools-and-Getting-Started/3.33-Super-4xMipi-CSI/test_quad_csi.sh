#!/usr/bin/env bash
# 四路 IMX219 CSI 摄像头测试脚本 (Jetson)
# 支持 nvarguscamerasrc 和 v4l2src 两种模式
# 用法: ./test_quad_csi.sh [check|preview|single|capture|info] [选项]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${SCRIPT_DIR}/captures"
NUM_BUFFERS="${NUM_BUFFERS:-30}"
TILE_W=320
TILE_H=240
CAP_W=640
CAP_H=480

# 摄像头模式: auto/nvargus/v4l2
CAMERA_MODE="${CAMERA_MODE:-auto}"

# V4L2 设备映射 (根据 v4l2-ctl --list-devices 自动检测)
declare -A V4L2_DEVICES

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

detect_display() {
    if [[ -n "${DISPLAY:-}" ]] && [[ -S "/tmp/.X11-unix/X${DISPLAY#:}" ]]; then
        return 0
    fi
    for d in :0 :1; do
        if [[ -S "/tmp/.X11-unix/X${d#:}" ]]; then
            DISPLAY="$d"
            export DISPLAY
            return 0
        fi
    done
    return 1
}

require_gstreamer() {
    if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
        echo -e "${RED}错误: 未找到 gst-launch-1.0${NC}" >&2
        exit 1
    fi
    if ! gst-inspect-1.0 nvarguscamerasrc >/dev/null 2>&1; then
        echo -e "${RED}错误: 未找到 nvarguscamerasrc 插件${NC}" >&2
        exit 1
    fi
}

check_hardware_access() {
    # 检查是否在容器/沙箱中运行
    if [[ -f "/.dockerenv" ]] || grep -q "docker\|lxc\|container" /proc/1/cgroup 2>/dev/null; then
        log_fail "检测到容器/沙箱环境"
        log_fail "摄像头需要直接访问硬件，请在宿主机上直接运行此脚本"
        exit 1
    fi
}

# 自动检测可用的摄像头后端
detect_camera_backend() {
    if [[ "$CAMERA_MODE" == "v4l2" ]]; then
        log_info "使用 V4L2 模式"
        CAMERA_BACKEND="v4l2"
        detect_v4l2_devices
        return 0
    fi

    if [[ "$CAMERA_MODE" == "nvargus" ]]; then
        if check_nvargus_devices; then
            log_info "使用 nvargus 模式"
            CAMERA_BACKEND="nvargus"
            return 0
        else
            log_fail "nvargus 模式需要 NVIDIA 设备节点"
            log_info "请设置 CAMERA_MODE=v4l2 使用 V4L2 模式"
            exit 1
        fi
    fi

    # auto 模式：优先尝试 nvargus，回退到 v4l2
    if check_nvargus_devices; then
        log_info "自动检测: nvargus 设备节点存在，测试可用性..."
        if test_nvargus_working; then
            log_info "nvargus 可用"
            CAMERA_BACKEND="nvargus"
        else
            log_warn "nvargus 设备节点存在但无法使用 (nvargus-daemon 可能异常)"
            log_info "尝试: sudo systemctl restart nvargus-daemon"
            log_info "切换到 V4L2 模式"
            CAMERA_BACKEND="v4l2"
            detect_v4l2_devices
        fi
    else
        log_warn "nvargus 不可用，切换到 V4L2 模式"
        CAMERA_BACKEND="v4l2"
        detect_v4l2_devices
    fi
}

# 检查 nvargus 所需的设备节点 (兼容 Jetson Nano/Xavier/Orin)
check_nvargus_devices() {
    # Orin: /dev/nvhost-ctrl-vi0, /dev/nvmap
    # Xavier/Nano: /dev/nvhost-ctrl, /dev/nvmap
    if [[ -e "/dev/nvmap" ]]; then
        if [[ -e "/dev/nvhost-ctrl" ]] || [[ -e "/dev/nvhost-ctrl-vi0" ]]; then
            return 0
        fi
    fi
    return 1
}

# 测试 nvargus 是否真正可用 (尝试打开 sensor-id=0)
test_nvargus_working() {
    local log
    log="$(mktemp)"
    if timeout 10 gst-launch-1.0 -q nvarguscamerasrc sensor-id=0 num-buffers=1 ! fakesink >"$log" 2>&1; then
        rm -f "$log"
        return 0
    fi
    rm -f "$log"
    return 1
}

# 检测 V4L2 摄像头设备
detect_v4l2_devices() {
    log_info "检测 V4L2 摄像头设备..."
    local found=0
    local -a i2c_addrs=()
    local -a vdevs=()

    # 查找 imx219 相关的 video 设备
    for vdev in /dev/video[0-9]*; do
        [[ -e "$vdev" ]] || continue
        local product
        product=$(udevadm info --query=all --name="$vdev" 2>/dev/null | grep "ID_V4L_PRODUCT" | cut -d= -f2 || true)
        if [[ "$product" == *"imx219"* ]] && [[ "$product" == *"vi-output"* ]]; then
            # 提取 I2C 地址部分 (如 "imx219 9-0010" -> "9-0010")
            local i2c_addr
            i2c_addr=$(echo "$product" | grep -oP '\d+-\d+' || echo "unknown")
            i2c_addrs+=("$i2c_addr")
            vdevs+=("$vdev")
            log_ok "发现摄像头: $vdev ($product)"
            ((found++)) || true
        fi
    done

    if [[ $found -eq 0 ]]; then
        log_warn "未找到 IMX219 V4L2 设备"
        log_info "尝试使用 /dev/video0, /dev/video2, /dev/video4, /dev/video6"
        V4L2_DEVICES[0]="/dev/video0"
        V4L2_DEVICES[1]="/dev/video2"
        V4L2_DEVICES[2]="/dev/video4"
        V4L2_DEVICES[3]="/dev/video6"
    else
        # 按 I2C 地址排序 (提取总线号进行排序)
        local -a sorted_indices=()
        for i in $(seq 0 $((found - 1))); do
            sorted_indices+=("$i")
        done

        # 冒泡排序按 I2C 地址
        for ((i = 0; i < found - 1; i++)); do
            for ((j = 0; j < found - i - 1; j++)); do
                local idx1=${sorted_indices[$j]}
                local idx2=${sorted_indices[$((j + 1))]}
                local bus1 bus2
                bus1=$(echo "${i2c_addrs[$idx1]}" | cut -d- -f1)
                bus2=$(echo "${i2c_addrs[$idx2]}" | cut -d- -f1)
                if [[ "$bus1" -gt "$bus2" ]]; then
                    sorted_indices[$j]=$idx2
                    sorted_indices[$((j + 1))]=$idx1
                fi
            done
        done

        # 按排序后的顺序填充 V4L2_DEVICES
        for ((i = 0; i < found && i < 4; i++)); do
            local idx=${sorted_indices[$i]}
            V4L2_DEVICES[$i]="${vdevs[$idx]}"
        done

        log_info "共找到 ${found} 个 IMX219 摄像头 (已按 I2C 总线排序)"
    fi
}

# 获取 sensor-id 对应的 V4L2 设备
get_v4l2_device() {
    local id="$1"
    if [[ ${#V4L2_DEVICES[@]} -eq 0 ]]; then
        detect_v4l2_devices
    fi

    # 尝试按索引获取
    local devices=("${V4L2_DEVICES[@]}")
    if [[ $id -lt ${#devices[@]} ]]; then
        echo "${devices[$id]}"
        return 0
    fi

    # 回退到默认映射
    local default_devices=("/dev/video0" "/dev/video2" "/dev/video4" "/dev/video6")
    if [[ $id -lt ${#default_devices[@]} ]] && [[ -e "${default_devices[$id]}" ]]; then
        echo "${default_devices[$id]}"
        return 0
    fi

    return 1
}

require_display() {
    if ! detect_display; then
        echo -e "${RED}错误: 未检测到 X11 显示，preview/single/capture 需要图形界面${NC}" >&2
        echo "  可先运行: export DISPLAY=:1" >&2
        exit 1
    fi
}

log_ok()   { echo -e "${GREEN}[OK]${NC} $*"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $*"; }
log_info() { echo -e "${CYAN}[INFO]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }

# 逐路打开摄像头并采集若干帧
test_sensor() {
    local id="$1"
    local buffers="${2:-$NUM_BUFFERS}"
    local log
    log="$(mktemp)"

    local pipeline=""
    if [[ "$CAMERA_BACKEND" == "nvargus" ]]; then
        pipeline="nvarguscamerasrc sensor-id=$id num-buffers=$buffers ! fakesink"
    else
        local vdev
        vdev=$(get_v4l2_device "$id")
        if [[ -z "$vdev" ]]; then
            log_fail "sensor-id=$id 未找到对应的 V4L2 设备"
            rm -f "$log"
            return 1
        fi
        pipeline="v4l2src device=$vdev num-buffers=$buffers ! video/x-raw,format=NV12,width=${CAP_W},height=${CAP_H} ! fakesink"
    fi

    DISPLAY="${DISPLAY:-:1}" timeout 20 gst-launch-1.0 -q $pipeline >"$log" 2>&1
    local exit_code=$?

    if [[ $exit_code -eq 0 ]] && grep -qi "done\|success\|eos" "$log"; then
        log_ok "sensor-id=$id 采集 ${buffers} 帧成功"
        rm -f "$log"
        return 0
    elif [[ $exit_code -eq 124 ]]; then
        log_fail "sensor-id=$id 超时 (20s)"
    else
        log_fail "sensor-id=$id 测试失败 (exit=$exit_code)"
    fi
    tail -n 8 "$log" >&2
    rm -f "$log"
    return 1
}

cmd_check() {
    local passed=0 failed=0
    log_info "开始逐路检测 sensor-id 0~3 (每路 ${NUM_BUFFERS} 帧)..."
    echo

    for id in 0 1 2 3; do
        if test_sensor "$id"; then
            ((passed++)) || true
        else
            ((failed++)) || true
        fi
    done

    echo
    echo "----------------------------------------"
    echo -e "结果: ${GREEN}${passed} 通过${NC}, ${RED}${failed} 失败${NC} / 共 4 路"
    [[ "$failed" -eq 0 ]]
}

# 2x2 四宫格实时预览
# compositor 需要系统内存；其后必须用 videoconvert，不能用 nvvidconv（会 NvVicCompose Failed）
cmd_preview() {
    require_display
    local mode="${PREVIEW_MODE:-cpu}"

    case "$mode" in
        cpu|sw|software)
            log_info "四路 CSI 2x2 预览 (software compositor, ${TILE_W}x${TILE_H})，按 Ctrl+C 退出"
            log_info "DISPLAY=$DISPLAY, 后端=$CAMERA_BACKEND"

            if [[ "$CAMERA_BACKEND" == "nvargus" ]]; then
                gst-launch-1.0 -e \
                    compositor name=comp \
                        sink_0::xpos=0 sink_0::ypos=0 \
                        sink_1::xpos="$TILE_W" sink_1::ypos=0 \
                        sink_2::xpos=0 sink_2::ypos="$TILE_H" \
                        sink_3::xpos="$TILE_W" sink_3::ypos="$TILE_H" \
                    ! "video/x-raw,format=I420,width=$((TILE_W * 2)),height=$((TILE_H * 2))" \
                    ! queue ! videoconvert ! ximagesink sync=false \
                    nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                        ! nvvidconv ! "video/x-raw,format=I420" \
                        ! videoscale ! "video/x-raw,width=${TILE_W},height=${TILE_H}" ! comp.sink_0 \
                    nvarguscamerasrc sensor-id=1 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                        ! nvvidconv ! "video/x-raw,format=I420" \
                        ! videoscale ! "video/x-raw,width=${TILE_W},height=${TILE_H}" ! comp.sink_1 \
                    nvarguscamerasrc sensor-id=2 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                        ! nvvidconv ! "video/x-raw,format=I420" \
                        ! videoscale ! "video/x-raw,width=${TILE_W},height=${TILE_H}" ! comp.sink_2 \
                    nvarguscamerasrc sensor-id=3 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                        ! nvvidconv ! "video/x-raw,format=I420" \
                        ! videoscale ! "video/x-raw,width=${TILE_W},height=${TILE_H}" ! comp.sink_3
            else
                local v0 v1 v2 v3
                v0=$(get_v4l2_device 0)
                v1=$(get_v4l2_device 1)
                v2=$(get_v4l2_device 2)
                v3=$(get_v4l2_device 3)
                gst-launch-1.0 -e \
                    compositor name=comp \
                        sink_0::xpos=0 sink_0::ypos=0 \
                        sink_1::xpos="$TILE_W" sink_1::ypos=0 \
                        sink_2::xpos=0 sink_2::ypos="$TILE_H" \
                        sink_3::xpos="$TILE_W" sink_3::ypos="$TILE_H" \
                    ! "video/x-raw,format=I420,width=$((TILE_W * 2)),height=$((TILE_H * 2))" \
                    ! queue ! videoconvert ! ximagesink sync=false \
                    v4l2src device="$v0" ! "video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1" \
                        ! videoscale ! "video/x-raw,format=I420,width=${TILE_W},height=${TILE_H}" ! comp.sink_0 \
                    v4l2src device="$v1" ! "video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1" \
                        ! videoscale ! "video/x-raw,format=I420,width=${TILE_W},height=${TILE_H}" ! comp.sink_1 \
                    v4l2src device="$v2" ! "video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1" \
                        ! videoscale ! "video/x-raw,format=I420,width=${TILE_W},height=${TILE_H}" ! comp.sink_2 \
                    v4l2src device="$v3" ! "video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1" \
                        ! videoscale ! "video/x-raw,format=I420,width=${TILE_W},height=${TILE_H}" ! comp.sink_3
            fi
            ;;
        nv|hw|nvmm)
            if [[ "$CAMERA_BACKEND" != "nvargus" ]]; then
                log_fail "nvcompositor 模式仅支持 nvargus 后端"
                exit 1
            fi
            if ! gst-inspect-1.0 nvcompositor >/dev/null 2>&1; then
                log_fail "未找到 nvcompositor，请用: PREVIEW_MODE=cpu $0 preview"
                exit 1
            fi
            log_info "四路 CSI 2x2 预览 (nvcompositor + nv3dsink)，按 Ctrl+C 退出"
            log_info "DISPLAY=$DISPLAY"
            gst-launch-1.0 -e \
                nvcompositor name=comp \
                    sink_0::xpos=0 sink_0::ypos=0 sink_0::width="$TILE_W" sink_0::height="$TILE_H" \
                    sink_1::xpos="$TILE_W" sink_1::ypos=0 sink_1::width="$TILE_W" sink_1::height="$TILE_H" \
                    sink_2::xpos=0 sink_2::ypos="$TILE_H" sink_2::width="$TILE_W" sink_2::height="$TILE_H" \
                    sink_3::xpos="$TILE_W" sink_3::ypos="$TILE_H" sink_3::width="$TILE_W" sink_3::height="$TILE_H" \
                ! "video/x-raw(memory:NVMM),format=RGBA,width=$((TILE_W * 2)),height=$((TILE_H * 2))" \
                ! nv3dsink sync=false \
                nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                    ! nvvidconv ! "video/x-raw(memory:NVMM),format=RGBA,width=${TILE_W},height=${TILE_H}" ! comp.sink_0 \
                nvarguscamerasrc sensor-id=1 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                    ! nvvidconv ! "video/x-raw(memory:NVMM),format=RGBA,width=${TILE_W},height=${TILE_H}" ! comp.sink_1 \
                nvarguscamerasrc sensor-id=2 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                    ! nvvidconv ! "video/x-raw(memory:NVMM),format=RGBA,width=${TILE_W},height=${TILE_H}" ! comp.sink_2 \
                nvarguscamerasrc sensor-id=3 ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
                    ! nvvidconv ! "video/x-raw(memory:NVMM),format=RGBA,width=${TILE_W},height=${TILE_H}" ! comp.sink_3
            ;;
        *)
            log_fail "未知 PREVIEW_MODE=$mode (可选: cpu / nv)"
            exit 1
            ;;
    esac
}

cmd_single() {
    local id="${1:-0}"
    require_display

    if [[ ! "$id" =~ ^[0-3]$ ]]; then
        echo -e "${RED}错误: sensor-id 必须是 0~3${NC}" >&2
        exit 1
    fi

    log_info "单路预览 sensor-id=$id，按 Ctrl+C 退出"
    if [[ "$CAMERA_BACKEND" == "nvargus" ]]; then
        gst-launch-1.0 -e \
            nvarguscamerasrc sensor-id="$id" ! "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1" \
            ! nvvidconv ! "video/x-raw,format=I420,width=${CAP_W},height=${CAP_H}" \
            ! videoconvert ! ximagesink sync=false
    else
        local vdev
        vdev=$(get_v4l2_device "$id")
        gst-launch-1.0 -e \
            v4l2src device="$vdev" ! "video/x-raw,width=${CAP_W},height=${CAP_H},framerate=30/1" \
            ! videoconvert ! ximagesink sync=false
    fi
}

capture_one() {
    local id="$1"
    local out="$2"
    local log
    log="$(mktemp)"

    local pipeline=""
    if [[ "$CAMERA_BACKEND" == "nvargus" ]]; then
        pipeline="nvarguscamerasrc sensor-id=$id num-buffers=1 ! nvvidconv ! video/x-raw,width=${CAP_W},height=${CAP_H} ! nvjpegenc ! filesink location=$out"
    else
        local vdev
        vdev=$(get_v4l2_device "$id")
        pipeline="v4l2src device=$vdev num-buffers=1 ! videoconvert ! video/x-raw,format=I420,width=${CAP_W},height=${CAP_H} ! jpegenc ! filesink location=$out"
    fi

    if DISPLAY="${DISPLAY:-:1}" timeout 15 gst-launch-1.0 -e -q $pipeline >"$log" 2>&1; then
        rm -f "$log"
        return 0
    fi
    tail -n 5 "$log" >&2
    rm -f "$log"
    return 1
}

cmd_capture() {
    require_display
    mkdir -p "$OUTPUT_DIR"
    local ts
    ts="$(date +%Y%m%d_%H%M%S)"
    local dir="${OUTPUT_DIR}/${ts}"
    mkdir -p "$dir"

    log_info "保存四路快照到 ${dir}/ (并行捕获)"
    local pids=()
    local files=()
    for id in 0 1 2 3; do
        files+=("${dir}/cam${id}.jpg")
        capture_one "$id" "${files[$id]}" &
        pids+=($!)
    done

    local passed=0 failed=0
    for i in 0 1 2 3; do
        if wait "${pids[$i]}"; then
            log_ok "cam${i}.jpg"
            ((passed++)) || true
        else
            log_fail "cam${i}.jpg"
            ((failed++)) || true
        fi
    done
    echo
    log_info "完成: ${passed} 成功, ${failed} 失败 | 目录: $dir"
    [[ "$failed" -eq 0 ]]
}

cmd_info() {
    echo "=== 系统信息 ==="
    echo "脚本目录: $SCRIPT_DIR"
    echo "DISPLAY:  ${DISPLAY:-未设置}"
    echo "CAMERA_MODE: ${CAMERA_MODE}"
    echo "CAMERA_BACKEND: ${CAMERA_BACKEND:-未检测}"
    detect_display && echo "自动检测 DISPLAY: $DISPLAY" || echo "未检测到 X11"
    echo

    echo "=== /dev/video* 设备 ==="
    local cam_count=0
    for vdev in /dev/video[0-9]*; do
        [[ -e "$vdev" ]] || continue
        local product caps
        product=$(udevadm info --query=all --name="$vdev" 2>/dev/null | grep "ID_V4L_PRODUCT" | cut -d= -f2 || echo "unknown")
        caps=$(udevadm info --query=all --name="$vdev" 2>/dev/null | grep "ID_V4L_CAPABILITIES" | cut -d= -f2 || echo "")
        if [[ "$caps" == *"capture"* ]]; then
            echo "  $vdev : $product [采集设备]"
            ((cam_count++)) || true
        else
            echo "  $vdev : $product"
        fi
    done
    echo "  共 ${cam_count} 个摄像头采集设备"
    echo

    echo "=== GStreamer 插件 ==="
    gst-inspect-1.0 nvarguscamerasrc 2>/dev/null | grep -E 'Long-name|Description' || true
    echo

    if [[ "$CAMERA_BACKEND" == "nvargus" ]]; then
        echo "=== Argus sensor-id 映射 ==="
        echo "  sensor-id=0 -> 左上"
        echo "  sensor-id=1 -> 右上"
        echo "  sensor-id=2 -> 左下"
        echo "  sensor-id=3 -> 右下"
    else
        echo "=== V4L2 设备映射 ==="
        for id in 0 1 2 3; do
            local vdev
            vdev=$(get_v4l2_device "$id" 2>/dev/null)
            echo "  sensor-id=$id -> $vdev"
        done
    fi
}

usage() {
    cat <<EOF
四路 CSI 摄像头测试 (IMX219 Quad)

用法:
  $0                     等同于 check
  $0 check               逐路检测 4 个 sensor (默认)
  $0 preview             2x2 四宫格实时预览 (默认 software compositor)
  $0 single <0-3>        单路预览
  $0 capture             四路各拍一张 JPEG (captures 也可)
  $0 info                显示设备与环境信息

环境变量:
  DISPLAY       X11 显示 (默认自动检测 :1 / :0)
  NUM_BUFFERS   check 模式每路采集帧数 (默认 30)
  PREVIEW_MODE  preview 模式: cpu (默认) 或 nv
  CAMERA_MODE   摄像头后端: auto (默认) / nvargus / v4l2

示例:
  $0 check
  $0 preview
  $0 single 2
  NUM_BUFFERS=10 $0 check
  CAMERA_MODE=v4l2 $0 check
EOF
}

check_camera_devices() {
    if ! ls /dev/video* >/dev/null 2>&1; then
        log_warn "未检测到 /dev/video* 设备"
        log_warn "请确认: 1) 摄像头已正确连接  2) 设备树已配置  3) 驱动已加载"
        return 1
    fi

    # 统计实际的摄像头采集设备 (排除 metadata 设备)
    # metadata 设备通常名称中包含 "metadata" 且没有 capture 能力
    local camera_count=0
    local total_count
    total_count=$(ls /dev/video* 2>/dev/null | wc -l)

    for vdev in /dev/video[0-9]*; do
        [[ -e "$vdev" ]] || continue
        local caps
        caps=$(udevadm info --query=all --name="$vdev" 2>/dev/null | grep "ID_V4L_CAPABILITIES" | cut -d= -f2 || true)
        if [[ "$caps" == *"capture"* ]]; then
            ((camera_count++)) || true
        fi
    done

    # 如果无法通过 udev 判断，使用经验法则：偶数号是采集设备
    if [[ $camera_count -eq 0 ]]; then
        camera_count=$((total_count / 2))
    fi

    log_info "检测到 ${total_count} 个 video 设备，其中 ${camera_count} 个摄像头采集设备"

    if [[ $camera_count -lt 4 ]]; then
        log_warn "预期 4 个摄像头，但只检测到 ${camera_count} 个"
    fi

    return 0
}

main() {
    require_gstreamer
    check_hardware_access
    detect_camera_backend
    detect_display || true
    check_camera_devices || true

    local cmd="${1:-check}"
    shift || true

    case "$cmd" in
        check|test)
            cmd_check
            ;;
        preview|quad)
            cmd_preview
            ;;
        single|one)
            cmd_single "${1:-0}"
            ;;
        capture|snap|photo|captures)
            cmd_capture
            ;;
        info|status)
            cmd_info
            ;;
        -h|--help|help)
            usage
            ;;
        *)
            echo -e "${RED}未知命令: $cmd${NC}" >&2
            usage
            exit 1
            ;;
    esac
}

main "$@"
