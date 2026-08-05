# YOLO26 模型训练示例
# 本脚本演示如何使用 Ultralytics YOLO26 的 Python API 训练自定义模型。
# 训练前请确保已准备好数据集并配置好 data.yaml 文件。
#
# 除了 Python API，也可以使用命令行（CLI）方式启动训练，两者功能等价：
#   yolo detect train data=<data.yaml路径> model=yolo26n.pt epochs=100 imgsz=640 device=0
#
# 训练完成后，模型文件会保存在 runs/detect/train/weights/ 目录下（best.pt 和 last.pt）。

from pathlib import Path

from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent


def train_model(
    data_yaml: str,
    model_name: str = "yolo26n.pt",
    epochs: int = 100,
    batch: int = 8,
    imgsz: int = 640,
    save_period: int = 5,
):
    """使用 YOLO26 Python API 训练模型。

    Args:
        data_yaml: 数据集配置文件 data.yaml 的路径
        model_name: 预训练模型名称，如 'yolo26n.pt'（轻量级）
                    如需更高精度可替换为 'yolo26s.pt'、'yolo26m.pt' 等
        epochs: 训练轮次
        batch: 批量大小
        imgsz: 输入图像尺寸
        save_period: 每隔多少个 epoch 保存一次模型检查点
    """
    # 加载预训练模型
    model = YOLO(model_name)

    # 启动训练
    results = model.train(
        data=data_yaml,
        batch=batch,
        epochs=epochs,
        imgsz=imgsz,
        save_period=save_period,
    )

    return results


if __name__ == "__main__":
    # 数据集 data.yaml 文件路径
    # 请将其替换为您自己的数据集配置文件路径
    data_yaml_path = str(SCRIPT_DIR / "dataset" / "data.yaml")

    # 如果数据集配置文件不存在，打印提示信息
    if not Path(data_yaml_path).exists():
        print(f"未找到数据集配置文件: {data_yaml_path}")
        print("请准备好数据集并配置好 data.yaml 文件后再运行训练。")
        print("也可使用 CLI 方式启动训练：")
        print("  yolo detect train data=<data.yaml路径> model=yolo26n.pt epochs=100 imgsz=640 device=0")
    else:
        train_model(data_yaml=data_yaml_path)
