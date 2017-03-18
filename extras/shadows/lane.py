import cv2
import numpy as np
import scipy

img=cv2.imread("l6.jpg")
img=cv2.resize(img, (640, 480), interpolation = cv2.INTER_AREA)
cv2.imshow("original image", img)

#contrast_fiter=np.asarray([[1, 1, 1], [0, 0, 0], [-1, -1, -1]])
#contrast_fiter=np.asarray([[1, 0, -1], [1, 0, -1], [1, 0, -1]])
contrast_fiter=np.asarray([[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]])
img_filtered=cv2.filter2D(img, -1, contrast_fiter)
cv2.imshow("vertical edges", img_filtered)

img_filtered_gray=cv2.cvtColor(img_filtered, code=cv2.COLOR_BGR2GRAY) 
cv2.imshow("vertical edges gray", img_filtered_gray)

lane_points=[]

def click_and_select(event, x, y, flags, param):
	global lane_points

	if event == cv2.EVENT_RBUTTONDBLCLK:
		lane_points.append((x, y))

cv2.namedWindow("select samples")
cv2.setMouseCallback("select samples", click_and_select)

while True:
	cv2.imshow("select samples", img_filtered)
	if cv2.waitKey(20) & 0xFF == 27:
		break

window_size=16
#for iter in range(len(lane_points)):
#	cv2.rectangle(img_filtered, (lane_points[iter][0]-10, lane_points[iter][1]-10), (lane_points[iter][0]+10, lane_points[iter][1]+10), (0, 255, 0), 2)
#cv2.imshow("recs", img_filtered)

#display selected windows as lane samples
#for iter in range(len(lane_points)):
	#cv2.rectangle(img_filtered_gray, (lane_points[iter][0]-window_size/2, lane_points[iter][1]-window_size/2), (lane_points[iter][0]+window_size/2, lane_points[iter][1]+window_size/2), 255, 2)

cv2.imshow("lane samples", img_filtered_gray)

lane_samples=[]
for iter in range(len(lane_points)):
	lane_samples.append(np.asarray(img_filtered_gray)[lane_points[iter][0]-window_size/2:lane_points[iter][0]+window_size/2, lane_points[iter][1]-window_size/2:lane_points[iter][1]+window_size/2])

signatures=[]
for iter in range(len(lane_samples)):
	signatures.append(abs(np.fft.fft2(lane_samples[iter])))

spectrum_match=np.zeros((img.shape[0], img.shape[1]))

for i in range(img.shape[0]-window_size):
	for j in range(img.shape[1]-window_size):
		#extract sample
		sample=np.asarray(img_filtered_gray)[i:i+window_size,j:j+window_size]
		sample_signature=abs(np.fft.fft2(sample))
		#match signatures
		for iter in range(len(lane_samples)):
			#print iter
			spectrum_match[i][j]=max(spectrum_match[i][j], np.sum(signatures[iter]*sample_signature)/5000000000)

#spectrum_match=cv2.fromarray(spectrum_match)

cv2.imshow("yay!", spectrum_match)





cv2.waitKey(0)


