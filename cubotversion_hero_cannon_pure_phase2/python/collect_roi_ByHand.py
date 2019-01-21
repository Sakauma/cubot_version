#encoding: utf-8
"""
手动标注工具
    b--上一页
    n--下一页
    s--保存
    q--退出
"""
import cv2

save_num = 0
img_suffix = ".png"
roi_save_path = "/home/snow/"
img_source_path = "/home/snow/Robomasters/documents/Sentry_NLG_18_5/images/"

drawing = False
isRoiOkFlag = True

ix_tl, iy_tl = -1, -1
ix_br, iy_br = -1, -1

def Draw_Rect_By_hand(event, x, y, flags, param):
    global ix_tl, iy_tl, ix_br, iy_br, drawing, isRoiOkFlag
    if event == cv2.EVENT_LBUTTONDOWN:
        if drawing == True:
            drawing = False
            isRoiOkFlag = True
            ix_br, iy_br = x, y
        else:
            drawing = True
            isRoiOkFlag = False
            ix_tl, iy_tl = x, y
    elif event == cv2.EVENT_RBUTTONDOWN or event == cv2.EVENT_MBUTTONDOWN:
        drawing = False
        ix_tl = -1
        iy_tl = -1
        isRoiOkFlag = False


k = 0
start_cnt = 25
cv2.namedWindow("img", 0)
cv2.setMouseCallback("img", Draw_Rect_By_hand)

while(1):
    BreakFlag= False
    isUpdata = True

    if start_cnt > 100:
        break
    if start_cnt < 0:
        start_cnt = 0

    if isUpdata:
        image_name = img_source_path + str(start_cnt) + img_suffix
        img = cv2.imread(image_name)
        if img is None:
            if k == ord('b'):
                start_cnt -= 1
            elif k == ord('n'):
                start_cnt += 1
            continue
        else:
            print (str(start_cnt))
            isUpdata = False
            ix_tl = -1
            iy_tl = -1

    while(1):
        img_copy = img.copy()

        if ix_tl != -1 and iy_tl != -1:
            cv2.circle(img_copy, (ix_tl, iy_tl), 5, (255, 0, 0), -1)
            if isRoiOkFlag:
                cv2.circle(img_copy, (ix_br, iy_br), 5, (0, 255, 0), -1)
                cv2.rectangle(img_copy, (ix_tl, iy_tl), (ix_br, iy_br), (0, 255, 0), 1)

                if iy_tl < iy_br and ix_tl < ix_br:
                    roi_img = img[iy_tl : iy_br, ix_tl : ix_br]
                elif iy_tl < iy_br and ix_tl > ix_br:
                    roi_img = img[iy_tl : iy_br, ix_br : ix_tl]
                elif iy_tl > iy_br and ix_tl < ix_br:
                    roi_img = img[iy_br : iy_tl, ix_tl : ix_br]
                else :
                    roi_img = img[iy_br : iy_tl, ix_br : ix_tl]

        cv2.imshow("img", img_copy)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('n'):
            start_cnt += 1
            isUpdata = True
            break
        elif k == ord('b'):
            start_cnt -= 1
            isUpdata = True
            break
        elif k == ord('q'):
            BreakFlag = True
            break
        elif k == ord('s'):
            if isRoiOkFlag:
                ix_tl = -1
                iy_tl = -1
                # roi_img = cv2.resize(roi_img, (40, 40))
                retval = cv2.imwrite(roi_save_path + str(save_num) + ".jpg", roi_img)
                if retval:
                    print "save roi successfully, img id is " + str(save_num) + "\n"
                save_num += 1

    if BreakFlag:
        print "exit the program\n"
        break
