#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import glob
import signal
import shutil
import subprocess
import urllib.request
import zipfile
import tempfile
import socket
import platform
import tarfile

VIDEO_DIR = "./videos"
RTSP_BASE_URL = "rtsp://127.0.0.1:8554"
NUM_STREAMS = 8
FFMPEG_CMD = "ffmpeg"
FFPROBE_CMD = "ffprobe"
MEDIAMTX_CONFIG = "./mediamtx.yml"
MEDIAMTX_VERSION = "1.8.0"
OS_NAME = platform.system().lower()
ARCH_NAME = platform.machine().lower()
IS_WINDOWS = OS_NAME.startswith("win")
IS_LINUX = OS_NAME.startswith("linux")
IS_MACOS = OS_NAME.startswith("darwin")
MEDIAMTX_EXE = "./mediamtx.exe" if IS_WINDOWS else "./mediamtx"

processes = []

def get_popen_kwargs():
    if IS_WINDOWS:
        return {"creationflags": subprocess.CREATE_NO_WINDOW}
    return {}

def launch_persistent_process(cmd):
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        **get_popen_kwargs()
    )
    return proc

def find_videos(directory, count):
    exts = ["*.mp4", "*.avi", "*.mkv", "*.mov", "*.flv", "*.ts"]
    files = []
    for ext in exts:
        files.extend(glob.glob(os.path.join(directory, ext)))
        files.extend(glob.glob(os.path.join(directory, ext.upper())))
    files = sorted(list(set(files)))
    return files[:count]

def get_mediamtx_download_info():
    if IS_WINDOWS:
        return (
            f"https://github.com/bluenviron/mediamtx/releases/download/v{MEDIAMTX_VERSION}/mediamtx_v{MEDIAMTX_VERSION}_windows_amd64.zip",
            "zip",
            "mediamtx.exe"
        )
    if IS_MACOS:
        if ARCH_NAME in ("x86_64",):
            arch = "amd64"
        elif ARCH_NAME in ("arm64",):
            arch = "arm64"
        else:
            raise RuntimeError(f"不支持的 macOS 架构: {ARCH_NAME}")
        return (
            f"https://github.com/bluenviron/mediamtx/releases/download/v{MEDIAMTX_VERSION}/mediamtx_v{MEDIAMTX_VERSION}_darwin_{arch}.tar.gz",
            "tar.gz",
            "mediamtx"
        )
    if IS_LINUX:
        if ARCH_NAME in ("x86_64", "amd64"):
            arch = "amd64"
        elif ARCH_NAME in ("aarch64", "arm64"):
            arch = "arm64"
        else:
            raise RuntimeError(f"不支持的 Linux 架构: {ARCH_NAME}")
        return (
            f"https://github.com/bluenviron/mediamtx/releases/download/v{MEDIAMTX_VERSION}/mediamtx_v{MEDIAMTX_VERSION}_linux_{arch}.tar.gz",
            "tar.gz",
            "mediamtx"
        )
    raise RuntimeError(f"不支持的操作系统: {platform.system()}")

def download_mediamtx(dest_path):
    url, archive_type, binary_name = get_mediamtx_download_info()
    print(f"下载 MediaMTX v{MEDIAMTX_VERSION} ...")
    print(f"  源: {url}")

    with tempfile.TemporaryDirectory() as tmpdir:
        archive_path = os.path.join(tmpdir, "mediamtx.archive")

        # 方式0: 检查当前目录是否有手动下载的压缩包（常见于网络不佳的情况）
        local_archive = None
        for candidate in ["mediamtx.tar.gz", "mediamtx.tgz", "mediamtx.zip"]:
            full = os.path.join(".", candidate)
            if os.path.exists(full) and os.path.getsize(full) > 1024:
                local_archive = full
                break
        if local_archive:
            print(f"  检测到本地压缩包: {os.path.basename(local_archive)}，直接解压...")
            archive_path = local_archive

        # 方式1: 优先用 curl
        curl_ok = local_archive is not None
        if not curl_ok and shutil.which("curl"):
            try:
                result = subprocess.run(
                    ["curl", "-L", "-o", archive_path, url],
                    timeout=300
                )
                curl_ok = (result.returncode == 0 and
                           os.path.exists(archive_path) and
                           os.path.getsize(archive_path) > 1024)
                if not curl_ok:
                    print(f"  curl 下载失败（返回码: {result.returncode}），尝试 urllib...")
            except subprocess.TimeoutExpired:
                print("  curl 下载超时（300秒），尝试 urllib...")
            except Exception as e:
                print(f"  curl 异常: {e}，尝试 urllib...")

        # 方式2: urllib 回退
        if not curl_ok:
            try:
                import urllib.request
                req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
                with urllib.request.urlopen(req, timeout=120) as resp:
                    with open(archive_path, "wb") as f:
                        while True:
                            chunk = resp.read(8192)
                            if not chunk:
                                break
                            f.write(chunk)
                curl_ok = (os.path.exists(archive_path) and
                           os.path.getsize(archive_path) > 1024)
            except Exception as e:
                raise RuntimeError(f"下载失败: {e}\n\n手动下载方法:\n  curl -L -o mediamtx.tar.gz {url}\n  tar xzf mediamtx.tar.gz mediamtx\n  chmod +x mediamtx")

        if not curl_ok:
            raise RuntimeError(f"下载的文件为空或损坏\n\n手动下载方法:\n  curl -L -o mediamtx.tar.gz {url}\n  tar xzf mediamtx.tar.gz mediamtx\n  chmod +x mediamtx")

        print("  解压中...")
        if archive_type == "zip":
            with zipfile.ZipFile(archive_path, "r") as z:
                z.extractall(tmpdir)
        else:
            with tarfile.open(archive_path, "r:gz") as t:
                t.extractall(tmpdir)
        src = os.path.join(tmpdir, binary_name)
        if not os.path.exists(src):
            raise RuntimeError("下载完成但未找到 MediaMTX 可执行文件")
        shutil.move(src, dest_path)
        if not IS_WINDOWS:
            os.chmod(dest_path, 0o755)
    print("MediaMTX 下载完成")

def get_lan_ip():
    try:
        if IS_LINUX:
            result = subprocess.run(['ip', 'route', 'get', '8.8.8.8'], capture_output=True, text=True, check=True)
            for line in result.stdout.split('\n'):
                if 'src' in line:
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == 'src':
                            ip = parts[i+1]
                            if ip != '127.0.0.1':
                                return ip
        elif IS_MACOS:
            result = subprocess.run(['ifconfig'], capture_output=True, text=True, check=True)
            for line in result.stdout.split('\n'):
                if 'inet ' in line and '127.0.0.1' not in line and '169.254' not in line:
                    parts = line.split()
                    idx = parts.index('inet')
                    return parts[idx + 1]
    except Exception:
        pass
    return "127.0.0.1"

def check_mediamtx_running():
    """检查 MediaMTX 进程是否已在运行"""
    if IS_WINDOWS:
        try:
            result = subprocess.run(
                ["tasklist", "/FI", f"IMAGENAME eq {os.path.basename(MEDIAMTX_EXE)}"],
                capture_output=True, text=True
            )
            return MEDIAMTX_EXE.lower() in result.stdout.lower() or \
                   os.path.basename(MEDIAMTX_EXE).lower() in result.stdout.lower()
        except:
            return False
    else:
        try:
            # 仅匹配当前用户启动的进程，避免误判
            result = subprocess.run(
                ["pgrep", "-u", str(os.getuid()), "-f", os.path.basename(MEDIAMTX_EXE)],
                capture_output=True, text=True
            )
            return result.returncode == 0
        except:
            return False

def check_mediamtx_binary_usable():
    """检查 mediamtx 二进制文件是否可在当前平台运行"""
    if not os.path.exists(MEDIAMTX_EXE):
        return False
    try:
        result = subprocess.run(
            [MEDIAMTX_EXE, "--help"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3
        )
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired, PermissionError):
        return False

def start_mediamtx():
    # 检查 MediaMTX 是否已经在运行
    if check_mediamtx_running() and check_port_listening(8554):
        print("MediaMTX 已在运行中，使用现有服务")
        # 返回一个伪进程对象，使其支持 poll() 方法
        class FakeProcess:
            def poll(self): return None
            def terminate(self): pass
            def wait(self, timeout=None): pass
            def kill(self): pass
        return FakeProcess()

    # 检查现有二进制是否可在当前平台运行，若不可用则重新下载
    if os.path.exists(MEDIAMTX_EXE) and not check_mediamtx_binary_usable():
        print(f"现有 mediamtx 二进制文件不兼容当前平台 ({OS_NAME}/{ARCH_NAME})，重新下载...")
        os.remove(MEDIAMTX_EXE)

    if not os.path.exists(MEDIAMTX_EXE):
        download_mediamtx(MEDIAMTX_EXE)
    with open(MEDIAMTX_CONFIG, "w", encoding="utf-8") as f:
        f.write("rtspAddress: 0.0.0.0:8554\n")
        f.write("rtspsAddress: 0.0.0.0:8322\n")
        f.write("paths:\n")
        f.write("  all:\n")
        f.write("    source: publisher\n")
    print("启动 MediaMTX ...")
    proc = launch_persistent_process([MEDIAMTX_EXE, MEDIAMTX_CONFIG])
    time.sleep(3)
    if proc.poll() is not None:
        # 如果启动失败，可能是端口占用
        if check_port_listening(8554):
             print("MediaMTX 启动后退出，但端口 8554 已被占用，假设已有服务在运行")
             class FakeProcess:
                 def poll(self): return None
                 def terminate(self): pass
                 def wait(self, timeout=None): pass
                 def kill(self): pass
             return FakeProcess()
        raise RuntimeError("MediaMTX 启动失败")
    print("MediaMTX 已启动，端口 8554")
    return proc

def start_ffmpeg_stream(video_path, stream_name):
    rtsp_url = f"{RTSP_BASE_URL}/{stream_name}"
    mode = "copy"
    copy_cmd = [
        FFMPEG_CMD,
        "-re",
        "-stream_loop", "-1",
        "-i", video_path,
        "-map", "0:v:0",      # 仅映射第一条视频流，忽略可能导致报错的数据流
        "-c:v", "copy",       # 直接复制视频流
        "-an", "-sn", "-dn",  # 无音频、无字幕、无数据流
        "-f", "rtsp",
        "-rtsp_transport", "tcp",
        rtsp_url
    ]
    print(f"推流: {stream_name} <- {os.path.basename(video_path)}")
    probe = subprocess.Popen(
        copy_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **get_popen_kwargs()
    )
    time.sleep(1.5)
    if probe.poll() is not None:
        print(f"⚠️ {stream_name} 原始编码推流被服务器拒绝(可能是缺少SDP参数)，自动尝试转码推流...")
        mode = "transcode"
        transcode_cmd = [
            FFMPEG_CMD,
            "-re",
            "-stream_loop", "-1",
            "-i", video_path,
            "-map", "0:v:0",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-vf", "scale=-2:720",
            "-b:v", "1500k",
            "-an", "-sn", "-dn",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            rtsp_url
        ]
        probe = subprocess.Popen(
            transcode_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            **get_popen_kwargs()
        )
        time.sleep(1.5)
        if probe.poll() is not None:
            _, stderr = probe.communicate()
            error_msg = stderr.decode('utf-8', errors='ignore')
            print(f"❌ {stream_name} 转码推流也失败，错误详情：\n{error_msg}")
            return None
        probe.terminate()
        try:
            probe.wait(timeout=2)
        except:
            probe.kill()
        proc = launch_persistent_process(transcode_cmd)
    else:
        probe.terminate()
        try:
            probe.wait(timeout=2)
        except:
            probe.kill()
        proc = launch_persistent_process(copy_cmd)
    time.sleep(0.6)
    if proc.poll() is not None:
        print(f"❌ {stream_name} 进程启动后立即退出")
        return None
    for _ in range(5):
        if check_rtsp_online(rtsp_url):
            break
        time.sleep(0.6)
    else:
        print(f"❌ {stream_name} 推流进程存在，但地址不可访问")
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except:
                proc.kill()
        return None
    if mode == "copy":
        print(f"✅ {stream_name} 推流成功 (copy)")
    else:
        print(f"✅ {stream_name} 推流成功 (transcode)")
    return proc

def check_rtsp_online(rtsp_url):
    cmd = [
        FFPROBE_CMD,
        "-rtsp_transport", "tcp",
        "-v", "error",
        "-show_streams",
        "-of", "compact",
        rtsp_url
    ]
    ret = subprocess.run(cmd, capture_output=True, text=True)
    return ret.returncode == 0 and ("codec_type=video" in ret.stdout)

def cleanup(signum=None, frame=None):
    print("\n停止所有进程...")
    for p in processes:
        if p.poll() is None:
            p.terminate()
    for p in processes:
        try:
            p.wait(timeout=5)
        except:
            p.kill()
    print("已清理所有进程")

def check_port_listening(port=8554):
    """检查指定端口是否被监听"""
    cmd = None
    if IS_LINUX:
        cmd = f"netstat -tlnp 2>/dev/null | grep :{port} || ss -tlnp 2>/dev/null | grep :{port}"
    elif IS_MACOS:
        cmd = f"netstat -an 2>/dev/null | grep LISTEN | grep '\\.{port}' || lsof -i :{port} 2>/dev/null | grep LISTEN"
    elif IS_WINDOWS:
        cmd = f"netstat -ano | findstr :{port}"

    if cmd:
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except Exception:
            return False
    return False

def main():
    # 检查 ffmpeg
    try:
        subprocess.run([FFMPEG_CMD, "-version"], capture_output=True, check=True)
    except:
        print("错误: ffmpeg 未安装或不在 PATH 中")
        sys.exit(1)
    try:
        subprocess.run([FFPROBE_CMD, "-version"], capture_output=True, check=True)
    except:
        print("错误: ffprobe 未安装或不在 PATH 中")
        sys.exit(1)

    videos = find_videos(VIDEO_DIR, NUM_STREAMS)
    if not videos:
        print(f"未在 '{VIDEO_DIR}' 找到视频文件")
        sys.exit(1)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        mediamtx_proc = start_mediamtx()
        if mediamtx_proc:
            processes.append(mediamtx_proc)
        
        time.sleep(2)
        if not check_port_listening(8554):
            print("⚠️  警告: MediaMTX 端口 8554 未监听，可能启动失败")
            print("   请检查以下问题:")
            print("   1. MediaMTX 二进制文件是否存在: %s" % MEDIAMTX_EXE)
            print("   2. 端口 8554 是否被其他应用占用")
            print("   3. 权限问题或系统限制")

        active_streams = []
        for i, vid in enumerate(videos[:NUM_STREAMS]):
            stream_name = f"cam{i+1}"
            proc = start_ffmpeg_stream(vid, stream_name)
            if proc is not None:
                processes.append(proc)
                active_streams.append(stream_name)
            time.sleep(0.5)

        print("\n========== RTSP 流信息 ==========")
        print("\n本地访问地址：")
        for stream_name in active_streams:
            print(f"  {stream_name}: {RTSP_BASE_URL}/{stream_name}")
        lan_ip = get_lan_ip()
        if lan_ip != "127.0.0.1":
            print(f"\n远程访问地址（局域网 IP: {lan_ip}）：")
            for stream_name in active_streams:
                print(f"  {stream_name}: rtsp://{lan_ip}:8554/{stream_name}")
        print("\n连通性检查：")
        for stream_name in active_streams:
            url = f"{RTSP_BASE_URL}/{stream_name}"
            ok = check_rtsp_online(url)
            if ok:
                print(f"  ✅ {stream_name} 可访问")
            else:
                print(f"  ❌ {stream_name} 不可访问")
        if not active_streams:
            print("  无可用推流，请检查视频文件编码格式或 ffmpeg 环境")
        print("\n================================")
        print("按 Ctrl+C 停止\n")

        while True:
            time.sleep(5)
            for p in processes:
                if p.poll() is not None:
                    if p == mediamtx_proc:
                        print("MediaMTX 意外退出")
                        cleanup()
    except KeyboardInterrupt:
        cleanup()
        sys.exit(0)
    except Exception as e:
        print(f"异常: {e}")
        cleanup()

if __name__ == "__main__":
    main()
