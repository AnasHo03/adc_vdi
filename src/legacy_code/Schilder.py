
import os
import time
import argparse
import subprocess

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

import sys
#import winsound
sys.path.append('/home/nano/mechatroniklabor_ws/src/tensorrt_demos')
sys.path.append('/home/nano/mechatroniklabor_ws/src/tensorrt_demos/utils')
sys.path.append('/home/nano/mechatroniklabor_ws/src/tensorrt_demos/plugins')


from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO


WINDOW_NAME = 'TrtYOLODemo'


def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)

    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-t', '--conf_thresh', type=float, default=0.8,
        help='set the detection confidence threshold')
    parser.add_argument(
        '-m', '--model', type=str, default='custom-yolov4-tiny',
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args


def loop_and_detect(cam, trt_yolo, conf_th, vis):

        img = cam.read()
        
        boxes, confs, clss = trt_yolo.detect(img, conf_th)
        
        print('Sign')
        with open('Schilder.txt','w') as f:
            f.write(str(clss))
            f.close
        with open('Schilder1.txt','w') as f:
            f.write(str(confs))
            f.close
        #winsound.Beep(2000,1500)
        print('Starting ParkingCrossRight')
        spawn_program_and_die(['python3', '/home/nano/mechatroniklabor_ws/src/wettbewerb/parkingCrossRight.py'])

def spawn_program_and_die(program, exit_code=0):
    """
    Start an external program and exit the script 
    with the specified return code.

    Takes the parameter program, which is a list 
    that corresponds to the argv of your command.
    """
    # Start the external program
    subprocess.Popen(program)
    # We have started the program, and can suspend this interpreter
    sys.exit(exit_code)

def main():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('/home/nano/mechatroniklabor_ws/src/tensorrt_demos/yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)
    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')  
    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)

    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    loop_and_detect(cam, trt_yolo, args.conf_thresh, vis=vis)




if __name__ == '__main__':
    main()
