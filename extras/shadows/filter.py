import cv2
import numpy as np
import scipy

img=cv2.imread("top1.jpg")
img_gray=cv2.cvtColor(img, code=cv2.COLOR_BGR2GRAY) 

print img_gray.shape

for i in range(img_gray.shape[0]):
	for j in range(img_gray.shape[1]/2):
		img_gray[i, j]=img_gray[i, j]*0.7


lane_width={5, 8, 10, 15, 20, 25, 30, 35}

#img_filtered=img_gray/100000
#contrast_fiter=np.zeros((1, 50))

'''lane_points=[]
def click_and_select(event, x, y, flags, param):
	#print 'check'
	global lane_points

	if event == cv2.EVENT_RBUTTONDBLCLK:
		print 'hahaha'
		lane_points.append((x, y))


cv2.namedWindow("find lane width")
cv2.setMouseCallback("find lane width", click_and_select)

while True:
	cv2.imshow("find lane width", img_gray)
	if cv2.waitKey(20) & 0xFF == 27:
		break


cv2.imshow("find lane width", img_gray)
cv2.waitKey(5000)
cv2.destroyWindow("find lane width")
cv2.waitKey(-1)
cv2.imshow("find lane width", img)
print lane_points'''


'''for width in lane_width:
	for i in range(50):
		if i>25-width/2 and i<25+width/2:
			contrast_fiter[0, i]=40
		else:
			contrast_fiter[0, i]=-20
	img_filtered=np.maximum(cv2.filter2D(img_gray, 0, contrast_fiter) , img_filtered)

img_filtered*=150'''

'''img_filtered = np.zeros((480,480), dtype=np.float64)

contrast_fiter=np.zeros((1, 25))
for i in range(25):
	if i>5 and i<20:
		contrast_fiter[0,i]=0.10
	else:
		contrast_fiter[0,i]=-0.12'''

img_filtered = np.zeros((480,480))

contrast_fiter=np.zeros((1, 25))
for i in range(25):
	if i>5 and i<20:
		contrast_fiter[0,i]=5
	else:
		contrast_fiter[0,i]=0



print contrast_fiter

#img_filtered=cv2.filter2D(img_gray, 0, contrast_fiter)
img_filtered=cv2.matchTemplate(img_gray, contrast_fiter, cv2.TM_CCOEFF_NORMED)
print img_filtered
print img_filtered.max()

low_values_indices = img_filtered < 30  # Where values are low
img_filtered[low_values_indices] = 0

high_values_indices = img_filtered > 0  # Where values are low
img_filtered[high_values_indices] = 255


cv2.imshow("sdsd", img_gray);
cv2.imshow("dsd", img_filtered)
cv2.waitKey(0)