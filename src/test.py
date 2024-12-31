import cv2
cap = cv2.VideoCapture("nvarguscamerasrc ! nvvidconv ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
ret, frame = cap.read()
cv2.imshow("Frame", frame)
cv2.waitKey(0)