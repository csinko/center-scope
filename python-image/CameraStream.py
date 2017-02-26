import numpy as np
import cv2
from imutils import contours
from skimage import measure
import time
import imutils
from Tkinter import *
from PIL import Image, ImageTk
import ttk
import serial

### IMPORTANT VARIABLES ###

WIDTH_PX = 640
HEIGHT_PX = 480
WIDTH_SEC = 26.9
HEIHT_SEC = 20.2
STEP = 10
THRESHOLD = 25
cap = cv2.VideoCapture(2) # 0=back camera; 1=front camera; 2=webcam

ser = serial.Serial(
        port='COM4',
        baudrate=19200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        #xonxoff=serial.XOFF,
        rtscts=False,
        #dsrdtr=False
        )

ser.isOpen()
print("Initializing Serial")
print("Write Command")
ser.write(bytearray([253,254,65,4,239]))
time.sleep(.05)
ser.write(bytearray([253,254,17,8,199]))
time.sleep(.05)
ser.write(bytearray([253,254,33,8,62]))
time.sleep(.05)
ser.write(bytearray([253,254,21,6,165,174,2,0,193]))
time.sleep(.05)
ser.write(bytearray([253,254,37,6,82,87,1,0,108]))
time.sleep(.05)
ser.write(bytearray([253,254,17,1,248]))
time.sleep(.3)
#ser.write(bytearray([253,254,35,4,20,188,199]))
time.sleep(.05)

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
time.sleep(3)
retv, img = cap.read()

while(img is None):
    time.sleep(0.5)
    retv, img = cap.read()

cv2.imshow('img', img)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#blurred = cv2.GaussianBlur(gray, (11, 11), 0)

thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)[1]

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
#cnts = contours.sort_contours(cnts)[0]

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
    print("Countours")
    print(selected_star_cnt)
    star_x, star_y, star_w, star_h = cv2.boundingRect(selected_star_cnt)
    print("starx, stary")
    print(star_x, star_y)

    root.destroy()
    print("Selected Star")
    print selected_star_cnt

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

def start_down():

    ## UP COMMAND
    ser.write(bytearray([253,254,35,2,0,1,131]))
    time.sleep(0.05)

def start_up():

    ## DOWN COMMAND
    ser.write(bytearray([253,254,35,2,255,255,160]))
    time.sleep(0.05)

def start_left():

    ## RIGHT COMMAND
    ser.write(bytearray([253,254,19,2,0,1,42]))
    time.sleep(0.05)

def start_right():

    ## LEFT COMMAND
    ser.write(bytearray([253,254,19,2,255,255,9]))
    time.sleep(0.05)

def stop_RL():
    #Left/Right Stop Command
    ser.write(bytearray([253,254,19,2,0,0,45]))
    time.sleep(0.05)
    print 'Stopping RL motor'

def stop_UD():
    #Up/Down Stop Command
    ser.write(bytearray([253,254,35,2,0,0,132]))
    time.sleep(0.05)
    print 'Stopping UD motor'
####################################################

### MONITOR MOVEMENT OF TELESCOPE ###

#star_x, star_y, star_w, star_h

center_x_min = (WIDTH_PX - THRESHOLD)/4
center_x_max = (WIDTH_PX + THRESHOLD)/4

center_y_min = (HEIGHT_PX - THRESHOLD)/4
center_y_max = (HEIGHT_PX + THRESHOLD)/4


lr_move = False
ud_move = False
print 'star_x, star_y'
print star_x, star_y

print 'center x min, center x max'
print center_x_min, center_x_max

print 'center y min, center y max'
print center_y_min, center_y_max

if star_x < center_x_min:
    #start_left()
    print 'right'
    start_right()
    lr_move = True

elif star_x > center_x_max:
    #start_right()
    print 'left'
    start_left()
    lr_move = True


if star_y < center_y_min:
    #start_down()
    print 'down'
    start_down()
    ud_move = True

elif star_y > center_y_max:
    #start_up()
    print 'up'
    start_up()
    ud_move = True


maxMean = None


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()


    # Display the resulting frame if it exists
    if(frame is None):
        # if something wonky happens with the crappy camera
        # wait for it to settle
        time.sleep(1)
    else:

        image = frame.copy()

        #print("Star x, stary")
        #print(star_x,star_y)
        cv2.rectangle(image, (star_x, star_y), (star_x + star_w, star_y + star_h), (0, 255, 0), 2)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

        thresh = cv2.dilate(thresh, None, iterations=4)

        for y in range(star_y - star_h, star_y + star_h, 5):
            for x in range(star_x - star_w, star_x + star_w, 5):
                mask = np.zeros(thresh.shape, dtype = 'uint8')
                cv2.circle(mask, (x, y), 10, 255, -1)
                mean = cv2.mean(thresh, mask = mask)[0]

                #If a new max mean is found, update the current max mean
                if maxMean is None or mean > maxMean[0]:
                    if (star_x - (star_w) <= x <= star_x + (star_w)) and (star_y - (star_h) <= y <= star_y + (star_h)):
                        maxMean = (mean, (x, y))

        star_x, star_y = maxMean[1]

        if center_x_min < star_x < center_x_max:
            print 'stop LR'
            stop_RL()
            lr_move = False

        if center_y_min < star_y < center_y_max:
            print 'stop UD'
            stop_UD()
            ud_move = False





        cv2.circle(image, maxMean[1], STEP, (150, 150, 255), 2)

        center_thresh_x_min = (WIDTH_PX - THRESHOLD)/2
        center_thersh_x_max = (WIDTH_PX + THRESHOLD)/2
        center_thresh_y_min = (HEIGHT_PX - THRESHOLD)/2
        center_thresh_y_max = (HEIGHT_PX + THRESHOLD)/2

        cv2.rectangle(image, (center_x_min, center_y_min), (center_x_max, center_y_max), (0, 255, 0))
        cv2.imshow('frame', image)


    if within_threshold(maxMean[1]):
        print("WITHIN THRESHOLD")
        break

    if not(ud_move or lr_move):
        print 'NOT MOVING'
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("C:\\Users\\Public\\Pictures\\cameraTest.jpg", frame)
        print("QUIT KEY")
        break

    maxMean = None

#END while(True):


# When everything done, release the capture

print("DONE!")

stop_UD()
stop_RL()
cap.release()
cv2.destroyAllWindows()
ser.close()
