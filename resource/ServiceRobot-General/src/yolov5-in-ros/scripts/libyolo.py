import sys
import time
from pathlib import Path

import cv2
import torch
import numpy as np
from typing import *
from colorama import Fore, Style


def red(msg: str, bold: bool = False) -> str:
    if bold:
        return f"{Fore.RED}{Style.BRIGHT}{msg}{Style.RESET_ALL}"
    return f"{Fore.RED}{msg}{Style.RESET_ALL}"


def green(msg: str, bold: bool = False) -> str:
    if bold:
        return f"{Fore.GREEN}{Style.BRIGHT}{msg}{Style.RESET_ALL}"
    return f"{Fore.GREEN}{msg}{Style.RESET_ALL}"


def yellow(msg: str, bold: bool = False) -> str:
    if bold:
        return f"{Fore.YELLOW}{Style.BRIGHT}{msg}{Style.RESET_ALL}"
    return f"{Fore.YELLOW}{msg}{Style.RESET_ALL}"


def blue(msg: str, bold: bool = False) -> str:
    if bold:
        return f"{Fore.BLUE}{Style.BRIGHT}{msg}{Style.RESET_ALL}"
    return f"{Fore.BLUE}{msg}{Style.RESET_ALL}"


yolo_path = Path(__file__).absolute().parent.joinpath("yolov5")
if yolo_path.__str__() not in sys.path:
    sys.path.append(
        yolo_path.__str__()
    )

from yolov5.utils.torch_utils import select_device
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.general import (
    non_max_suppression, scale_coords, xyxy2xywh
)
from yolov5.models.experimental import attempt_load
from yolov5.utils.augmentations import letterbox


class Config(object):
    # 使用GPU
    device = select_device("0")
    # NMS 超参数
    conf_thres = 0.25
    iou_thres = 0.45
    classes = None
    agnostic = False
    multi_label = False
    max_det = 1000
    classes = None
    # 显示参数
    linethickness = 3
    img_size = 640


class YoloV5s(Config):
    def __init__(self, weight_path: Union[str, Path] = None) -> None:
        """
        Notes:
            ROS调用YOLOV5S的中间件，作者：jackwang
        Arguments:
            weight_path (Union[str, Path]): 权重参数的路径，指定为None则为默认路径，默认路径为当前功能包下的model文件夹，
                权重参数需要命名为yolov5s.pt，大小写敏感
        Returns:
            None
        """
        # 权重参数
        weight_path: Path = Path(__file__).resolve().parent.parent.joinpath(
            "models", "best.pt") if weight_path is None else Path(weight_path)
        assert weight_path.exists(), f"权重参数路径不存在"
        super().__init__()
        self._load(weight_path)

    def _load(self, weight_path: Union[str, Path]) -> None:
        """
        Notes:
            YOLOV5S的加载函数，在推断前必须被调用
        """
        self.model = attempt_load(
            weights=weight_path, device=self.device)
        self.stride = int(self.model.stride.max())

    def _preprocess(self, img: np.ndarray, if_show: bool = True) -> Tuple[np.ndarray]:
        """
        Notes:
            YOLOV5S的图像预处理函数，主要对图像添加了batch并且调整了维度
        Arguments:
            img (np.ndarrar): 需要预处理的图像
            if_show (bool): 是否返回处理前的图像
        Returns:
            Tuple[np.ndarray, np.ndarray | None]: 预处理后的图像和原图像，原图像用于显示
        """
        assert img.shape[-1] == 3, red(f"图像通道数量不对: {img.shape}")
        origin = None
        if if_show:
            origin = img.copy()
        # 添加batch维度
        img: np.ndarray = letterbox(img, self.img_size, stride=self.stride)[0]
        img = np.expand_dims(img, axis=0)
        # BGR转RGB同时调整至Batch, Channel, Height, Width
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)
        return img, origin

    def inference(self, img: np.ndarray, if_absolute: bool = False, if_show: bool = True) -> Tuple[Dict[str, List[Tuple[float]]], np.ndarray]:
        """
        Notes:
            YOLOV5S的推断函数，修改超参数则修改Config类即可，注意为了实时性选择一次只推断一张图像
        Arguments:
            img (np.ndarray):  需要推断的图像，颜色通道为BGR，图像为[Height, Width, Channel]
            if_absolute (bool): 是否返回检测框的像素值，如果为Fasle则返回检测框除以图像大小之后的相对值
            if_show (bool): 是否显示绘制检测框之后的图像
        Returns:
            Dict[str, List[Tuple[List, float]]]:: 字典中每个键为类别，列表中每一个元素表示检测到该类别的物体，每一个物体以Tuple表示，
            每一个Tuple中有五个元素，分别为检测狂的左上角x值，左上角的y值，检测框的宽度，检测框的高度，以及置信
            np.ndarray: 绘制后的图
        Example:
            >>> import cv2, pprint
            >>> success_flag, img = cv2.VideoCapture(0).read()
            >>> result, drawed_img = YoloV5s().inference(img)
            >>> pprint.pprint(result)
            {
                'chair': [
                        (
                            0.9343750476837158,
                            0.8114583492279053,
                            0.12812501192092896,
                            0.29374998807907104,
                            0.4897725284099579
                        )
                    ],
                'person': [
                        (
                            0.45781248807907104,
                            0.7729166746139526,
                            0.30937501788139343,
                            0.44583332538604736,
                            0.82495671510696416
                        ),
                        (
                            0.6703125238418579,
                            0.6552083492279053,
                            0.653124988079071,
                            0.6895833015441895,
                            0.8768638372421265
                        )
                    ]
            }
        """
        img: torch.Tensor
        origin_img: np.ndarray
        img, origin_img = self._preprocess(img=img)

        img = torch.from_numpy(img).to(self.device) / 255

        pred = self.model(img, False, False)[0]
        pred = non_max_suppression(
            prediction=pred, conf_thres=self.conf_thres, iou_thres=self.iou_thres,
            classes=self.classes, agnostic=self.agnostic, max_det=self.max_det
        )

        results = {}

        # 命令行输出信息
        info_str = f"At {time.ctime()}, "  # print string
        info_str += '%gx%g input, ' % img.shape[2:]
        # pred一个batch中所有图像的预测第一个维度为batch
        # 正常情况下只有一个预测
        for i, det in enumerate(pred):
            gn = torch.tensor(origin_img.shape)[
                [1, 0, 1, 0]]  # normalization gain whwh
            # 获得 class name
            names = self.model.modules.names if hasattr(
                self.model, "module") else self.model.names
            # 画图工具
            if if_show:
                annotator = Annotator(
                    im=origin_img, line_width=self.linethickness, example=str(names))

            # 如果有预测
            if len(det):
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], origin_img.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    # add to string
                    info_str += green(
                        f"{n} {names[int(c)]}{'s' * (n > 1)}") + ", "

                print(info_str)

                # 循环画框
                # xyxy左上角，右下角的xyxy，置信度，类别信息
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    label = f"{names[c]} {conf:.2f}"
                    # 对图像大小进行相对化
                    if not if_absolute:
                        xywh = torch.tensor(xyxy).view(1, 4)
                    else:
                        xywh = torch.tensor(xyxy).view(1, 4) / gn
                    # 转换
                    xywh = (xyxy2xywh(xywh)).view(-1).tolist()
                    try:
                        results[names[c]].append((*xywh, float(conf)))
                    except KeyError:
                        results[names[c]] = [(*xywh, float(conf))]
                    line = cls, *xywh, conf
                    if if_show:
                        annotator.box_label(xyxy, label, color=colors(c, True))
        if if_show:
            return results, annotator.result()
        else:
            return results


if __name__ == "__main__":
    cap = cv2.VideoCapture(2)
    yolo = YoloV5s()
    while cv2.waitKey(1) != 27 and (re := cap.read())[0]:
        yolo.inference(img=re[1], if_absolute=True)
