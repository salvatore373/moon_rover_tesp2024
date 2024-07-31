import cv2
import numpy as np

#NOTES
#PRESS Q TO QUIT!!!
#This runs currently as a standalone script, code will need a bit of adaptation but could be turned into a function pretty easily, everything before
#the while true loop would be part of the init function, everything in the loop would be a function call to process one frame

#To read in single image as file
#imRaw = cv2.imread("ref.jpg")

#This opens the webcam
vid = cv2.VideoCapture(0) 

ret, imRaw = vid.read()
(sourceH, sourceW, c) = imRaw.shape[:3]
sf = 1   
#print(type(im))
#print("h: ", h)
#print("w: ", w)
#print("c: ", c)

gx = 0
gy = 0
gw = 0
gh = 0
rx = 0
ry = 0
rw = 0
rh = 0


def rgbFilter(im, targetColor, colorTol):
    targetColor = targetColor[::-1]
    targetLow = [x - colorTol for x in targetColor]
    targetHi = [x + colorTol for x in targetColor]
    targetLow = np.clip(targetLow,0,255)
    targetHi = np.clip(targetHi,0,255)
    targetLow = np.array(targetLow, dtype = "uint8")
    targetHi = np.array(targetHi, dtype = "uint8")

    mask = cv2.inRange(im, targetLow, targetHi)
    output = cv2.bitwise_and(im, im, mask = mask)
    return output

rectMap = None
redTgt = [250,22,24] #NOT IMPLEMENTED IDIOT
grnTgt = [1,159,115]

while(True):
    ret, imRaw = vid.read()
    imRaw = cv2.flip(imRaw, 1)
    im = cv2.resize(imRaw, (int(sf*sourceW),int(sf*sourceH)))

    rfilt = rgbFilter(im, [189,47,33], 40)
    gfilt = rgbFilter(im, [0,178,123], 60)

    gray = cv2.cvtColor(gfilt, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 3)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    thresh = cv2.threshold(sharpen, 160, 255, cv2.THRESH_BINARY)[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 50
    max_area = 1500

 
  
    #MAKE THIS A FUNCTION LATER
    gray = cv2.cvtColor(rfilt, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 3)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    thresh = cv2.threshold(sharpen, 160, 255, cv2.THRESH_BINARY)[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnts2 = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = cnts2[0] if len(cnts2) == 2 else cnts2[1]

    if rectMap is None:
        rectMap = cv2.bitwise_and(im,cv2.bitwise_not(im)) #This is fucking stupid btw

    if (len(cnts) != 0) and (len(cnts2) != 0):
        grnCont = max(cnts, key = cv2.contourArea)
        redCont = max(cnts2, key = cv2.contourArea)
        if cv2.contourArea(grnCont) > min_area and cv2.contourArea(redCont) > min_area:
            rectMap = cv2.bitwise_and(im,cv2.bitwise_not(im)) #This is fucking stupid btw
            gx,gy,gw,gh = cv2.boundingRect(grnCont)
            rx,ry,rw,rh = cv2.boundingRect(redCont)
            cv2.rectangle(rectMap, (gx, gy), (gx + gw, gy + gh), (36,255,12), 2)
            cv2.rectangle(rectMap, (rx, ry), (rx + rw, ry + rh), (36,12,255), 2)
            cgx = gx + int(gw/2)
            cgy = gy + int(gh/2)
            crx = rx + int(rw/2)
            cry = ry + int(rh/2)
            cx = int((cgx + crx)/2)
            cy = int((cgy + cry)/2)

            #Green on right, Red on left
            thetaR = np.arctan2((cgx-crx),(cgy-cry))
            l2x = int(40*np.cos(-1*thetaR))
            l2y = int(40*np.sin(-1*thetaR))

            cv2.line(rectMap, (cgx, cgy), (l2x + cgx, l2y + cgy),(255,255,255),1)
            cv2.line(rectMap, (crx, cry), (l2x + crx, l2y + cry),(255,255,255),1)
            cv2.putText(rectMap, str(round(thetaR*180/np.pi,3)), (100,100), cv2.FONT_HERSHEY_SIMPLEX,  0.75, (255,255,255), 2, cv2.LINE_AA) 

    # show the images
    outimg = cv2.cvtColor(close, cv2.COLOR_GRAY2BGR)
    cv2.imshow("images", np.hstack([im, outimg, rectMap]))

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

# When everything done, release the capture
vid.release()
cv2.destroyAllWindows()