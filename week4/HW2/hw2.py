import cv2
import numpy as np

# 1. Read the images
image1 = cv2.imread("./images/left.jpg")
image2 = cv2.imread("./images/right.jpg")

# 2. Convert the images to Grayscale
image1_gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

# 3. Find feature points with SIFT
sift = cv2.SIFT_create()
keypoints1, descriptors1 = sift.detectAndCompute(image1_gray, None)
keypoints2, descriptors2 = sift.detectAndCompute(image2_gray, None)

left_keypoints = cv2.drawKeypoints(image1_gray, keypoints1, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
right_keypoints = cv2.drawKeypoints(image2_gray, keypoints2, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

left_feature = np.hstack((image1, left_keypoints))      # 依照投影片範例，將原圖與特徵圖左右拼接
right_feature = np.hstack((image2, right_keypoints))    # 原圖在左，標有特徵點的灰階圖在右

cv2.imwrite("./left_feature.jpg", left_feature)
cv2.imwrite("./right_feature.jpg", right_feature)

# 4. Matching features with knn match
bf = cv2.BFMatcher(cv2.NORM_L2)
matches = bf.knnMatch(descriptors1, descriptors2, k=2)

good_matches = []                       # 透過設定門檻值篩選出較好的配對
for m, n in matches:
    if m.distance < 0.5 * n.distance:   # 經過測試，至少要設在 0.58 以下，才不會在前一百個配對中出現肉眼可見的錯誤
        good_matches.append(m)
good_matches = sorted(good_matches, key=lambda x: x.distance)

# 將前 100 個 Match 到的特徵點連線，只取 100 是因為太多會導致畫面很亂，難以用肉眼檢查
image_matches = cv2.drawMatches(image1, keypoints1, image2, keypoints2, good_matches[:100], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
cv2.imwrite("./feature_matching.jpg", image_matches)

# 5. Compute homography matrix (Hint: cv2.findHomography())
points1 = np.zeros((len(good_matches), 2), dtype=np.float32)
points2 = np.zeros((len(good_matches), 2), dtype=np.float32)
for i, match in enumerate(good_matches):
    points1[i, :] = keypoints1[match.queryIdx].pt
    points2[i, :] = keypoints2[match.trainIdx].pt

homography_matrix, mask = cv2.findHomography(points2, points1, cv2.RANSAC)

# 6. Perspective Transformation (Hint: cv2.warpPerspective())
height2, width2 = image2.shape[:2]
corners2 = np.float32([[0, 0], [0, height2], [width2, height2], [width2, 0]]).reshape(-1, 1, 2)
transformed_corners = cv2.perspectiveTransform(corners2, homography_matrix)

all_corners = np.concatenate((transformed_corners, np.float32([[0, 0], [0, image1.shape[0]], [image1.shape[1], image1.shape[0]], [image1.shape[1], 0]]).reshape(-1, 1, 2)), axis=0)
[x_min, y_min] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
[x_max, y_max] = np.int32(all_corners.max(axis=0).ravel() + 0.5)

translation_dist = [-x_min, -y_min]
homography_matrix_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])
image2_aligned = cv2.warpPerspective(image2, homography_matrix_translation @ homography_matrix, (x_max - x_min, y_max - y_min))

# 7. Combine images
stitched_image = np.zeros((y_max - y_min, x_max - x_min, 3), dtype=np.uint8)    # 建立能放得下放拼接圖片後的背景圖
stitched_image[translation_dist[1]:image1.shape[0] + translation_dist[1], translation_dist[0]:image1.shape[1] + translation_dist[0]] = image1
stitched_image = np.maximum(stitched_image, image2_aligned)                     # 將 left.jpg 作為基底和與其對齊過後的 right.jpg 合併
cv2.imwrite("./result.jpg", stitched_image)