# Edge Impulse - OpenMV FOMO Object Detection Example
#
# This work is licensed under the MIT license.
# Copyright (c) 2013-2024 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor, image, time, tf, math, uos, gc, lcd
from pyb import Pin
sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 240))       # Set 240x240 window.
sensor.skip_frames(time=2000)          # Let the camera adjust.
sensor.set_hmirror(True)
sensor.set_vflip(True)

lcd.init()

led = Pin(("rpi-p505",0x505))
net = None
labels = None
nowtime = 0
lasttime = 0
min_confidence = 0.85
k = 0
j = 236934720
try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
]

threshold_list = [(math.ceil(min_confidence * 255), 255)]

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh

    scale = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

    l = [[] for i in range(oc)]

    for i in range(oc):
        img = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img.find_blobs(
            threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1
        )
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = (
                img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
            )
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            l[i].append((x, y, w, h, score))
    return

clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    led.value(0)
    nowtime = time.time()                              #获取当前时刻
    local_time = time.localtime(nowtime+ j)
    for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
        if i == 0: continue  # background class
        if len(detection_list) == 0: continue  # no detections for this class?

        print("********** %s **********" % labels[i])
        for d in detection_list:

            if (nowtime - lasttime) >= 2:k = 0         #超过2s未识别到对象自动清空识别次数
            k = k + 1
            [x, y, w, h] = d.rect()
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            print('x %d\ty %d' % (center_x, center_y))
            print("k = ",k)                            #显示当前识别次数
            img.draw_circle((center_x, center_y, 12), color=colors[i])
            lasttime = time.time()                     #获取上一次识别对象时刻
            error_time = time.localtime(lasttime+ j )
            print("异常状态：", error_time)

            if i == 1:print("fire")
            if k >= 9:k = 0                            #识别次数超过9清零
            if k >= 7:led.value(1)                     #识别超过7次认为识别有效，蜂鸣器报警


    lcd.display(img)
    print("当前时间：", local_time)
    print(clock.fps(), "fps", end="\n\n")
