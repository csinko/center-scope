import numpy as np
import cv2
from imutils import contours
from skimage import measure
import time
import imutils
from Tkinter import *
from PIL import Image, ImageTk
import ttk

### IMPORTANT VARIABLES ###

WIDTH_PX = 640
HEIGHT_PX = 480
WIDTH_SEC = 26.9
HEIHT_SEC = 20.2
STEP = 10
THRESHOLD = 10
cap = cv2.VideoCapture(0) # 0=back camera; 1=front camera; 2=webcam
##################################################

### FUNCTIONS ###
def within_threshold((x, y)):
    #global WIDTH_PX
    #global HEIGHT_PX
    #global THRESHOLD

    if ((WIDTH_PX/2)-THRESHOLD <= x <= (WIDTH_PX/2)+THRESHOLD) and ((HEIGHT_PX/2)-THRESHOLD <= y <= (HEIGHT_PX/2)+THRESHOLD) :
        return True

    return False

#END within_threshold

##################################################

### FIND ALL POSSIBLE STARS ###
print 'FIND STARS'

retv, img = cap.read()

while(not retv):
    time.sleep(0.5)
    retv, img = cap.read()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#blurred = cv2.GaussianBlur(gray, (11, 11), 0)

thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

thresh = cv2.dilate(thresh, None, iterations=4)

cv2.imshow('thresh', thresh)

labels = measure.label(thresh, neighbors=8, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

for label in np.unique(labels):
    if label == 0:
        continue

    labelMask = np.zeros(thresh.shape, dtype="uint8")
    labelMask[labels == label] = 255
    numPixels = cv2.countNonZero(labelMask)

    if numPixels > 3:
        mask = cv2.add(mask, labelMask)


cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]
cnts = contours.sort_contours(cnts)[0]

count = 1
for (i, c) in enumerate(cnts):
    (x, y, w, h) = cv2.boundingRect(c)
    ((cX, cY), radius) = cv2.minEnclosingCircle(c)
    print count, '(', int(cX), ', ', int(cY), ')'
    cv2.circle(img, (int(cX), int(cY)),  int(radius), (0, 0, 255), 3)
    cv2.putText(img, "#{}".format(i+1), (x, y - 15), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 1)
    count += 1

cv2.imwrite('test.png', img)

print 'END FIND STARS'

##################################################

### DISPLAY USER INTERFACE ###

print 'DISPLAY INTERFACE'

selected_star = None

values = []

for i in range(0, len(cnts)):
    values.append(i+1)


b, g, r = cv2.split(img)
img = cv2.merge((r, g, b))

root = Tk(className = "Star Select")

cboCombo = ttk.Combobox( root, values=values,state="readonly",  textvariable="Select Star...")
star_x, star_y, star_w, star_h = (None, None, None, None)

def on_click():
    global star_x, star_y, star_w, star_h

    selected_star_cnt = cnts[int(cboCombo.get())-1] # (i, c) touple [see below]
    star_x, star_y, star_w, star_h = cv2.boundingRect(selected_star_cnt)

    root.destroy()
    print selected_star

b = Button(root, text=" CONFIRM ", command=on_click)


im = Image.fromarray(img)
imgtk = ImageTk.PhotoImage(image=im)

star_img = ttk.Label(root, image=imgtk)

star_img.pack()
cboCombo.pack( side='left', anchor='w', padx=12, pady=8 )
b.pack()

root.mainloop()

print 'END DISPLAY INTERFACE'

##################################################

### TELESCOPE MOVEMENT METHODS ###

def start_up():
    print 'up'

def start_down():
    print 'down'

def start_right():
    print 'right'

def start_left():
    print 'left'

def stop_RL():
    print 'Stopping RL motor'

def stop_UD():
    print 'Stopping UD motor'

####################################################

### MONITOR MOVEMENT OF TELESCOPE ###

#star_x, star_y, star_w, star_h

center_x_min = (WIDTH_PX - THRESHOLD)/2
center_x_max = (WIDTH_PX + THRESHOLD)/2

center_y_min = (HEIGHT_PX - THRESHOLD)/2
center_y_max = (HEIGHT_PX + THRESHOLD)/2


if star_x < center_x_min:
    start_left()

elif star_X > center_x_max:
    start_right()


if star_y < center_y_min:
    start_down()

elif star_y > center_y_max:
    start_up()



maxMean = None



while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()


    # Display the resulting frame if it exists
    if(frame is None):
        # if something wonky happens with the crappy camera
        # wait for it to settle
        time.sleep(10)
    else:

        image = frame.copy()
        cv2.rectangle(image, (star_x, star_y), (star_x + star_w, star_y + star_h), (0, 255, 0), 2)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

        thresh = cv2.dilate(thresh, None, iterations=4)

        #### NEED TO CHANGE THIS ########################
        for y in range(star_y - star_h, star_y + star_h, 2):
            for x in range(star_x - star_w, star_x + star_w, 2):
                mask = np.zeros(thresh.shape, dtype = 'uint8')
                cv2.circle(mask, (x, y), 10, 255, -1)
                mean = cv2.mean(thresh, mask = mask)[0]

                #If a new max mean is found, update the current max mean
                if maxMean is None or mean > maxMean[0]:
                    if (star_x - (star_w) <= x <= star_x + (star_w)) and (star_y - (star_h) <= y <= star_y + (star_h)):
                        maxMean = (mean, (x, y))
        ##################################################

        star_x, star_y = maxMean[1]

        cv2.circle(image, maxMean[1], STEP, (150, 150, 255), 2)
        cv2.rectangle(image, ((WIDTH_PX-THRESHOLD)/2, (HEIGHT_PX-THRESHOLD)/2), ((WIDTH_PX+THRESHOLD)/2, (HEIGHT_PX+THRESHOLD)/2), (0, 255, 0))
        cv2.imshow('frame', image)



    if within_threshold(maxMean[1]):
        print("WITHIN THRESHOLD")
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("C:\\Users\\Public\\Pictures\\cameraTest.jpg", frame)
        print("QUIT KEY")
        break

    maxMean = None

#END while(True):


# When everything done, release the capture

print("DONE!")

cap.release()
cv2.destroyAllWindows()
