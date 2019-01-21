#encoding: utf-8

import cv2
import os
import numpy as np

save_num = 0
end_num = 100
img_suffix = ".png"
roi_save_path = "/home/snow/"
img_source_path = "/home/snow/Robomasters/documents/Sentry_NLG_18_5/images/"

isSaveFlag = True
waitKeyVal = 0
x_step, y_step = 50, 25
roi_w, roi_h = 50, 50

k = 0
img_cnt = 0

while(1):
    BreakFlag = False
    if img_cnt > end_num or img_cnt < 0:
        break

    if k == ord('n'):
        img_cnt += 1
    elif k == ord('b'):
        img_cnt -= 1

    print "img_cnt is " + str(img_cnt)
    src_img = cv2.imread(img_source_path + str(img_cnt) + img_suffix)
    if src_img is None:
        continue

    src_img = cv2.resize(src_img, (0, 0), None, 0.5, 0.5, cv2.INTER_LINEAR)
    shape = src_img.shape
    h, w, channels = shape[0], shape[1], shape[2]
    print w, h, channels

    # loop
    for i in range(0, w, x_step):
        if i > w - roi_w:
            continue
        for j in range(0, h, y_step):
            if j > h - roi_h:
                continue
            x_l, y_l = i, j
            x_r, y_r = i + roi_w, j + roi_h
            roi = src_img[y_l : y_r, x_l : x_r]

            img_copy = src_img.copy()
            cv2.rectangle(img_copy, (x_l, y_l), (x_r, y_r), (0, 255, 0), 1)

            cv2.imshow("roi", roi)
            cv2.imshow("img_copy", img_copy)
            k = cv2.waitKey(waitKeyVal)

            if isSaveFlag:
                roi = cv2.resize(roi, (40, 40))
                isSaveSuccFlag = cv2.imwrite(roi_save_path + str(save_num) + ".jpg", roi)
                cv2.waitKey(1)
                if isSaveSuccFlag == True:
                    save_num += 1

            if k == ord('q'):
                os.exit(0)
            elif k == ord('n'):
                img_cnt += 1
                BreakFlag = True
                break
            elif k == ord('b'):
                img_cnt -= 1
                BreakFlag = True
                break
        if BreakFlag:
            break
