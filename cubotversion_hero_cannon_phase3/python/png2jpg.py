import cv2

img_root = "/home/snow/Desktop/"
src_suffix = ".png"

save_root = "/home/snow/Desktop/"
dst_suffix = ".jpg"

cnt = 0
start_num = 0
while (1):
    if cnt > 100:
        break

    src_img = cv2.imread(img_root + str(cnt) + src_suffix)
    cnt += 1
    if src_img is None:
        continue

    cv2.imwrite(save_root + str(cnt) + dst_suffix, src_img)
