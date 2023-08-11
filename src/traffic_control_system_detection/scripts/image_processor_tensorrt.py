import numpy as np
import cv2
import os
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import random
import matplotlib.pyplot as plt

from PIL import Image
from numpy import ndarray
from typing import List, Tuple, Union
from pathlib import Path

TRT_LOGGER = trt.Logger()

#https://github.com/triple-Mu/YOLOv8-TensorRT


CLASSES = ('1','cross_parking','overtaking_allowed','overtaking_forbidden','parallel_parking','pit_in','pit_out')
# colors per classes

COLORS = {
    cls: [random.randint(0, 255) for _ in range(3)]
    for i, cls in enumerate(CLASSES)
}

def blob(im: ndarray, return_seg: bool = False) -> Union[ndarray, Tuple]:
    seg = None
    if return_seg:
        seg = im.astype(np.float32) / 255
    im = im.transpose([2, 0, 1])
    im = im[np.newaxis, ...]
    im = np.ascontiguousarray(im).astype(np.float32) / 255
    if return_seg:
        return im, seg
    else:
        return im

def letterbox(im: ndarray,
              new_shape: Union[Tuple, List] = (640, 640),
              color: Union[Tuple, List] = (114, 114, 114)) \
        -> Tuple[ndarray, float, Tuple[float, float]]:
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    # new_shape: [width, height]

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[1], new_shape[1] / shape[0])
    # Compute padding [width, height]
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[0] - new_unpad[0], new_shape[1] - new_unpad[
        1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im,
                            top,
                            bottom,
                            left,
                            right,
                            cv2.BORDER_CONSTANT,
                            value=color)  # add border
    return im, r, (dw, dh)

def det_postprocess(data: Tuple[ndarray, ndarray, ndarray, ndarray]):
    assert len(data) == 4
    num_dets, bboxes, scores, labels = (i[0] for i in data)
    nums = num_dets.item()
    if nums == 0:
        return np.empty((0, 4), dtype=np.float32), np.empty(
            (0, ), dtype=np.float32), np.empty((0, ), dtype=np.int32)
    bboxes = bboxes[:nums]
    scores = scores[:nums]
    labels = labels[:nums]
    return bboxes, scores, labels

SUFFIXS = ('.bmp', '.dng', '.jpeg', '.jpg', '.mpo', '.png', '.tif', '.tiff',
           '.webp', '.pfm')

def path_to_list() -> List:
    images_path = Path("/home/johannes/Desktop/Projekte/VDIADC/Training environment/gitrepo/adc/src/frame_samples_zed")
    assert images_path.exists()
    if images_path.is_dir():
        images = [
            i.absolute() for i in images_path.iterdir() if i.suffix in SUFFIXS
        ]
    else:
        assert images_path.suffix in SUFFIXS
        images = [images_path.absolute()]
    return images



def main() -> None:

    from pycuda_api import TRTEngine

    Engine = TRTEngine()
    H, W = Engine.inp_info[0].shape[-2:]

    images = path_to_list()

    for image in images:
        bgr = cv2.imread(str(image))
        draw = bgr.copy()
        bgr, ratio, dwdh = letterbox(bgr, (W, H))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        tensor = blob(rgb, return_seg=False)
        dwdh = np.array(dwdh * 2, dtype=np.float32)
        tensor = np.ascontiguousarray(tensor)
        # inference
        data = Engine(tensor)

        bboxes, scores, labels = det_postprocess(data)
        if bboxes.size == 0:
            # if no bounding box
            print(f'{image}: no object!')
            continue
        bboxes -= dwdh
        bboxes /= ratio

        for (bbox, score, label) in zip(bboxes, scores, labels):
            bbox = bbox.round().astype(np.int32).tolist()
            cls_id = int(label)
            cls = CLASSES[cls_id]
            color = COLORS[cls]
            cv2.rectangle(draw, bbox[:2], bbox[2:], color, 2)
            cv2.putText(draw,
                        f'{cls}:{score:.3f}', (bbox[0], bbox[1] - 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, [225, 255, 255],
                        thickness=2)

        cv2.imshow('result', draw)
        cv2.waitKey(0)



if __name__ == '__main__':
    main()