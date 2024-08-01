import cv2
import numpy as np

#NOTES
#PRESS Q TO QUIT!!!
#This runs currently as a standalone script, code will need a bit of adaptation but could be turned into a function pretty easily, everything before
#the while true loop would be part of the init function, everything in the loop would be a function call to process one frame

#To read in single image as file
#imRaw = cv2.imread("ref.jpg")

#print(type(im))
#print("h: ", h)
#print("w: ", w)
#print("c: ", c)
class tagFinder:

    def __init__(self):
        self.gx = 0
        self.gy = 0
        self.gw = 0
        self.gh = 0
        self.rx = 0
        self.ry = 0
        self.rw = 0
        self.rh = 0
        self.cx = 0
        self.cy = 0
        self.thetaR = 0
        self.rectMap = None
        self.dispImg = None
        self.optimized = False
        self.tagsFound = False

    def rgbFilter(self, im, targetColor, colorTol):
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


    def tagAngle(self, im, redTarget = (250,22,24), redBand = 40, greenTarget = (1,159,115), greenBand = 60):

        #Ressurect in case configurable version breaks
        #rfilt = self.rgbFilter(im, (189,47,33), 40) 
        #gfilt = self.rgbFilter(im, (0,178,123), 60)

        rfilt = self.rgbFilter(im, redTarget, redBand)
        gfilt = self.rgbFilter(im, greenTarget, greenBand)

        #Green Processing
        gray = cv2.cvtColor(gfilt, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 3)
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

        thresh = cv2.threshold(sharpen, 160, 255, cv2.THRESH_BINARY)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        min_area = 40
        max_area = 1500

        #Red Processing
        gray = cv2.cvtColor(rfilt, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 3)
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

        thresh = cv2.threshold(sharpen, 160, 255, cv2.THRESH_BINARY)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        close2 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts2 = cv2.findContours(close2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = cnts2[0] if len(cnts2) == 2 else cnts2[1]

        #Outputs
        if self.rectMap is None:
            self.rectMap = cv2.bitwise_and(im,cv2.bitwise_not(im)) #This is fucking stupid btw
        self.tagsFound = False
        if (len(cnts) != 0) and (len(cnts2) != 0):

            grnCont = max(cnts, key = cv2.contourArea)
            redCont = max(cnts2, key = cv2.contourArea)
            if cv2.contourArea(grnCont) > min_area and cv2.contourArea(redCont) > min_area:
                self.tagsFound = True
                self.gx,self.gy,self.gw,self.gh = cv2.boundingRect(grnCont)
                self.rx,self.ry,self.rw,self.rh = cv2.boundingRect(redCont)
                
                self.cgx = self.gx + int(self.gw/2)
                self.cgy = self.gy + int(self.gh/2)
                self.crx = self.rx + int(self.rw/2)
                self.cry = self.ry + int(self.rh/2)
                self.cx = int((self.cgx + self.crx)/2)
                self.cy = int((self.cgy + self.cry)/2)

                #Green on right, Red on left
                self.thetaR = np.arctan2((self.cgx-self.crx),(self.cgy-self.cry))
                l2x = int(40*np.cos(-1*self.thetaR))
                l2y = int(40*np.sin(-1*self.thetaR))
                if self.optimized == False:
                    self.rectMap = cv2.bitwise_and(im,cv2.bitwise_not(im)) #This is fucking stupid btw
                    cv2.rectangle(self.rectMap, (self.gx, self.gy), (self.gx + self.gw, self.gy + self.gh), (36,255,12), 2)
                    cv2.rectangle(self.rectMap, (self.rx, self.ry), (self.rx + self.rw, self.ry + self.rh), (36,12,255), 2)
                    cv2.line(self.rectMap, (self.cgx, self.cgy), (l2x + self.cgx, l2y + self.cgy),(255,255,255),1)
                    cv2.line(self.rectMap, (self.crx, self.cry), (l2x + self.crx, l2y + self.cry),(255,255,255),1)
                    cv2.putText(self.rectMap, str(round(self.thetaR*180/np.pi,3)), (100,100), cv2.FONT_HERSHEY_SIMPLEX,  0.75, (255,255,255), 2, cv2.LINE_AA) 
        if self.dispImg is None:
            self.dispImg = cv2.cvtColor(cv2.bitwise_or(close,close2), cv2.COLOR_GRAY2BGR)
        elif self.optimized == False:
            self.dispImg = cv2.cvtColor(cv2.bitwise_or(close,close2), cv2.COLOR_GRAY2BGR)
        
        return ((self.cx,self.cy,round(self.thetaR*180/np.pi,3)),self.tagsFound,self.dispImg, self.rectMap)


if __name__ == "__main__":
    #This opens the webcam
    vid = cv2.VideoCapture(0) 

    ret, imRaw = vid.read()
    (sourceH, sourceW, c) = imRaw.shape[:3]
    sf = 1  
    tf = tagFinder() 
    tf.optimized = False
    while(True):
        ret, imRaw = vid.read()
        imRaw = cv2.flip(imRaw, 1)
        im = cv2.resize(imRaw, (int(sf*sourceW),int(sf*sourceH)))
        ((x,y,theta),_,outimg,rectMap) = tf.tagAngle(im)
        print(theta)
        cv2.imshow("images", np.hstack([im, outimg, rectMap]))
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

    # When everything done, release the capture
    vid.release()
    cv2.destroyAllWindows()


        