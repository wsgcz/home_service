import cv2
import numpy as np
import insightface
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image

handler = insightface.model_zoo.get_model('/home/linxi/Videos/buffalo_m (1)/buffalo_m/w600k_r50.onnx')
handler.prepare(ctx_id=0)
face=handler.get("t1.jpg")
face=ins_get_image('t1')
