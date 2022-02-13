#!/usr/bin/env python
# coding: utf-8

# In[30]:


import numpy as np
import imutils
import cv2
from scipy import ndimage
from scipy.linalg import *
from scipy import *
import random
import math

class Panaroma:

    def image_stitch(self, images, lowe_ratio=0.75, max_Threshold=4.0,match_status=False):

        #detect the features and keypoints from SIFT
        (imageB, imageA) = images
        (KeypointsA, features_of_A) = self.Detect_Feature_And_KeyPoints(imageA)
        (KeypointsB, features_of_B) = self.Detect_Feature_And_KeyPoints(imageB)

        #got the valid matched points
        Values = self.matchKeypoints(KeypointsA, KeypointsB,features_of_A, features_of_B, lowe_ratio, max_Threshold)

        if Values is None:
            return None

        #to get perspective of image using computed homography
        (matches, Homography) = Values
        result_image = self.getwarp_perspective(imageA,imageB,Homography)
        cv2.imwrite("panaroma/result.jpg",result_image)
        result_image[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
        cv2.imwrite("panaroma/result1.jpg",result_image)
        
        return result_image
    def calculateHomography(self, correspondences):
        #loop through correspondences and create assemble matrix
        aList = []
        for corr in correspondences:
            p1 = np.matrix([corr.item(0), corr.item(1), 1])
            p2 = np.matrix([corr.item(2), corr.item(3), 1])

            a2 = [0, 0, 0, -p2.item(2) * p1.item(0), -p2.item(2) * p1.item(1), -p2.item(2) * p1.item(2),
                  p2.item(1) * p1.item(0), p2.item(1) * p1.item(1), p2.item(1) * p1.item(2)]
            a1 = [-p2.item(2) * p1.item(0), -p2.item(2) * p1.item(1), -p2.item(2) * p1.item(2), 0, 0, 0,
                  p2.item(0) * p1.item(0), p2.item(0) * p1.item(1), p2.item(0) * p1.item(2)]
            aList.append(a1)
            aList.append(a2)

        matrixA = np.matrix(aList)

        #svd composition
        u, s, v = np.linalg.svd(matrixA)

        #reshape the min singular value into a 3 by 3 matrix
        h = np.reshape(v[8], (3, 3))

        #normalize and now we have h
        h = (1/h.item(8)) * h
        return h
    #
    #Calculate the geometric distance between estimated points and original points
    #
    def geometricDistance(self, correspondence, h):

        p1 = np.transpose(np.matrix([correspondence[0].item(0), correspondence[0].item(1), 1]))
        estimatep2 = np.dot(h, p1)
        estimatep2 = (1/estimatep2.item(2))*estimatep2

        p2 = np.transpose(np.matrix([correspondence[0].item(2), correspondence[0].item(3), 1]))
        error = p2 - estimatep2
        return np.linalg.norm(error)
    #
    #Runs through ransac algorithm, creating homographies from random correspondences
    #
    def ransac(self, corr, thresh):
        maxInliers = []
        finalH = None
        for i in range(1000):
            #find 4 random points to calculate a homography
            corr1 = corr[random.randrange(0, len(corr))]
            corr2 = corr[random.randrange(0, len(corr))]
            randomFour = np.vstack((corr1, corr2))
            corr3 = corr[random.randrange(0, len(corr))]
            randomFour = np.vstack((randomFour, corr3))
            corr4 = corr[random.randrange(0, len(corr))]
            randomFour = np.vstack((randomFour, corr4))

            #call the homography function on those points
            h = self.calculateHomography(randomFour)
            inliers = []

            for i in range(len(corr)):
                d = self.geometricDistance(corr[i], h)
                if d < 5:
                    inliers.append(corr[i])

            if len(inliers) > len(maxInliers):
                maxInliers = inliers
                finalH = h
            #print ("Corr size: ", len(corr), " NumInliers: ", len(inliers), "Max inliers: ", len(maxInliers))

            if len(maxInliers) > (len(corr)*thresh):
                break
        return finalH, maxInliers


    def calculateCornerX(self, pixelHeight, pixelWidth, Homography):
        newX = ((Homography[0, 0]*pixelWidth+Homography[0, 1]*pixelHeight+Homography[0, 2])/(Homography[2, 0]*pixelWidth+Homography[2, 1]*pixelHeight+Homography[2, 2]))
        return newX
    def calculateCornerY(self, pixelHeight,pixelWidth, Homography):
        newY = ((Homography[1, 0]*pixelWidth+Homography[1, 1]*pixelHeight+Homography[1, 2])/(Homography[2, 0]*pixelWidth+Homography[2, 1]*pixelHeight+Homography[2, 2]))
        return newY
        
        
        
    def warpImage(self,imageA, Homography, width, height):
        newimage = np.zeros((height, width, 3))
        #Homography = np.linalg. inv(H)
       # print(newimage.shape)
       # print(" 0 ",imageA.shape[0]," 1 ", imageA.shape[1])
       # print(Homography.shape)
       # print(type(Homography))
        count = 0
        cornerX=[]
        cornerY=[]
        cornerX.append(self.calculateCornerX(0,0,Homography))
        cornerY.append(self.calculateCornerY(0,0,Homography))
        cornerX.append(self.calculateCornerX(0,imageA.shape[1],Homography))
        cornerY.append(self.calculateCornerY(0,imageA.shape[1],Homography))
        cornerX.append(self.calculateCornerX(imageA.shape[0],0,Homography))
        cornerY.append(self.calculateCornerY(imageA.shape[0],0,Homography))
        cornerX.append(self.calculateCornerX(imageA.shape[0],imageA.shape[1],Homography))
        cornerY.append(self.calculateCornerY(imageA.shape[0],imageA.shape[1],Homography))
        min_x=min(cornerX)
        min_y=min(cornerY)
        max_x=max(cornerX)
        max_y=max(cornerY)
        invHomography=np.linalg.inv(Homography)
        for pheight in range(round(min_y),round(max_y)):
            for pwidth in range(round(min_x),round(max_x)):
                pointX= round(self.calculateCornerX(pheight,pwidth,invHomography))
                pointY= round(self.calculateCornerY(pheight,pwidth,invHomography))
                for k in range(3):
                    if(pwidth>=width or pheight>=height or pwidth<0 or pheight<0):
                            continue
                    if(pointX>=imageA.shape[1] or pointY>=imageA.shape[0] or pointX<0 or pointY<0):
                        continue
                    newimage[pheight][pwidth][k] = imageA[pointY][pointX][k]
        
        ''' for pixelHeight in range(imageA.shape[0]): #400  
            for pixelWidth in range(imageA.shape[1]): #603
                #newX = round((Homography[0, 0]*pixelHeight+Homography[0, 1]*pixelWidth+Homography[0, 2])/(Homography[2, 0]*pixelHeight+Homography[2, 1]*pixelWidth+Homography[2, 2]))
                #newY = round((Homography[1, 0]*pixelHeight+Homography[1, 1]*pixelWidth+Homography[1, 2])/(Homography[2, 0]*pixelHeight+Homography[2, 1]*pixelWidth+Homography[2, 2]))
                newX = ((Homography[0, 0]*pixelWidth+Homography[0, 1]*pixelHeight+Homography[0, 2])/(Homography[2, 0]*pixelWidth+Homography[2, 1]*pixelHeight+Homography[2, 2]))
                newY = ((Homography[1, 0]*pixelWidth+Homography[1, 1]*pixelHeight+Homography[1, 2])/(Homography[2, 0]*pixelWidth+Homography[2, 1]*pixelHeight+Homography[2, 2]))
                newX1=round(newX)
                newY1=round(newY)
                newX2=math.floor(newX)
                newY2=math.floor(newY)
                newX3=math.ceil(newX)
                newY3=math.ceil(newY)
                for k in range(3):
                    if(newX1>=width or newY1>=height or newX1<0 or newY1<0) or (newX2>=width or newY2>=height or newX2<0 or newY2<0) or (newX3>=width or newY3>=height or newX3<0 or newY3<0):
                        continue
                    else:
                        if(imageA[pixelHeight][pixelWidth][k]==0):
                            continue
                        newimage[newY1][newX1][k] = imageA[pixelHeight][pixelWidth][k]
                        newimage[newY2][newX2][k] = imageA[pixelHeight][pixelWidth][k]
                        newimage[newY3][newX3][k] = imageA[pixelHeight][pixelWidth][k]''' 
       
                        
                        
                    
        return newimage
        
    def getwarp_perspective(self,imageA,imageB,Homography):
        val = imageA.shape[1] + imageB.shape[1]        
        result_image = self.warpImage(imageA, Homography, val , imageA.shape[0])
        return result_image

    def Detect_Feature_And_KeyPoints(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # detect and extract features from the image
        descriptors = cv2.xfeatures2d.SIFT_create()
        (Keypoints, features) = descriptors.detectAndCompute(image, None)

        Keypoints = np.float32([i.pt for i in Keypoints])
        return (Keypoints, features)

    def get_Allpossible_Match(self,featuresA,featuresB):

        # compute the all matches using euclidean distance and opencv provide
        #DescriptorMatcher_create() function for that
        match_instance = cv2.DescriptorMatcher_create("BruteForce")
        All_Matches = match_instance.knnMatch(featuresA, featuresB, 2)

        return All_Matches

    def All_validmatches(self,AllMatches,lowe_ratio):
        #to get all valid matches according to lowe concept..
        valid_matches = []

        for val in AllMatches:
            if len(val) == 2 and val[0].distance < val[1].distance * lowe_ratio:
                valid_matches.append((val[0].trainIdx, val[0].queryIdx))

        return valid_matches

    def Compute_Homography(self,pointsA,pointsB,max_Threshold):
        #to compute homography using points in both images
        
        correspondenceList = []
        for i in range(len(pointsA)):
            x1 = pointsA[i][0]
            y1 = pointsA[i][1]
            x2 = pointsB[i][0]
            y2 = pointsB[i][1]
            correspondenceList.append([x1, y1, x2, y2])
        

        corrs = np.matrix(correspondenceList)
        finalH, inliers = self.ransac(corrs, max_Threshold)
        return finalH

    def matchKeypoints(self, KeypointsA, KeypointsB, featuresA, featuresB,lowe_ratio, max_Threshold):

        AllMatches = self.get_Allpossible_Match(featuresA,featuresB);
        valid_matches = self.All_validmatches(AllMatches,lowe_ratio)

        if len(valid_matches) > 4:
            # construct the two sets of points
            pointsA = np.float32([KeypointsA[i] for (_,i) in valid_matches])
            pointsB = np.float32([KeypointsB[i] for (i,_) in valid_matches])

            Homograpgy = self.Compute_Homography(pointsA, pointsB, max_Threshold)
            return (valid_matches, Homograpgy)
        else:
            return None

    def get_image_dimension(self,image):
        (h,w) = image.shape[:2]
        return (h,w)

    def get_points(self,imageA,imageB):

        (hA, wA) = self.get_image_dimension(imageA)
        (hB, wB) = self.get_image_dimension(imageB)
        vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
        vis[0:hA, 0:wA] = imageA
        vis[0:hB, wA:] = imageB

        return vis


    def draw_Matches(self, imageA, imageB, KeypointsA, KeypointsB, matches, status):

        (hA,wA) = self.get_image_dimension(imageA)
        vis = self.get_points(imageA,imageB)

        # loop over the matches
        for ((trainIdx, queryIdx), s) in zip(matches, status):
            if s == 1:
                ptA = (int(KeypointsA[queryIdx][0]), int(KeypointsA[queryIdx][1]))
                ptB = (int(KeypointsB[trainIdx][0]) + wA, int(KeypointsB[trainIdx][1]))
                cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

        return vis


# In[2]:


def find_mid(no_of_images):
    if(no_of_images%2)==0:
        return no_of_images/2
    return (no_of_images+1)/2


# In[32]:


import copy
def crop(IMG_IN):
    # keep a copy of original image
    original = cv2.imread(IMG_IN)

    # Read the image, convert it into grayscale, and make in binary image for threshold value of 1.
    img = cv2.imread(IMG_IN,0)

    # use binary threshold, all pixel that are beyond 3 are made white
    _, thresh_original = cv2.threshold(img, 3, 255, cv2.THRESH_BINARY)

    # Now find contours in it.
    thresh = copy.copy(thresh_original)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # get contours with highest height
    lst_contours = []
    for cnt in contours:
        ctr = cv2.boundingRect(cnt)
        lst_contours.append(ctr)
    x,y,w,h = sorted(lst_contours, key=lambda coef: coef[3])[-1]

    crop = original[y:y+h,x:x+w-4]
    cv2.imwrite(IMG_IN,crop)


# In[34]:


import imutils
import cv2

#Take picture from folder like: Hill1 & Hill2, scene1 & scene2, my1 & my2, taj1 & taj2, lotus1 & lotus2, beach1 & beach2, room1 & room2
print("Enter the number of images you want to concantenate:")
no_of_images = int(input())
print("Enter the number of images you want to concantenate at a time:")
m = int(input())
filename = []
mid_image = int(find_mid(no_of_images))
for i in range(no_of_images):
    imageName = "intersection/"+str(i)+".jpg"
    filename.append(imageName)

images = []

for i in range(no_of_images):
    images.append(cv2.imread(filename[i]))

#We need to modify the image resolution and keep our aspect ratio use the function imutils
for i in range(no_of_images):
    images[i] = imutils.resize(images[i], width=400)

for i in range(no_of_images):
    images[i] = imutils.resize(images[i], height=400)
    
total_iterations = no_of_images 
print("total number of iterations ", total_iterations)
panaroma = Panaroma()

'''
    
(result1, matched_points) = panaroma.image_stitch([images[mid_image-1], images[mid_image]], match_status=True)
cv2.imwrite("panaroma/Panorama_image"+str(mid_image-1)+str(mid_image)+".jpg",result1)
crop("panaroma/Panorama_image"+str(mid_image-1)+str(mid_image)+".jpg")
result1 = cv2.imread("panaroma/Panorama_image"+str(mid_image-1)+str(mid_image)+".jpg")
print(mid_image)
for i in range(mid_image, no_of_images-2):
    (result1, matched_points) = panaroma.image_stitch([result1, images[i+1]], match_status=True)
    cv2.imwrite("panaroma/Panorama_image"+str(mid_image)+str(i+1)+".jpg",result1)
    crop("panaroma/Panorama_image"+str(mid_image)+str(i+1)+".jpg")
    cv2.imread("panaroma/Panorama_image"+str(mid_image)+str(i+1)+".jpg")
    result1 = cv2.imread("panaroma/Panorama_image"+str(mid_image)+str(i+1)+".jpg")
    
return  
(r, matched_points) = panaroma.image_stitch([images[mid_image-1], images[mid_image-2]], match_status=True)
cv2.imwrite("panaroma/Panorama_image"+str(mid_image-1)+str(mid_image-2)+".jpg",r)
crop("panaroma/Panorama_image"+str(mid_image-1)+str(mid_image-2)+".jpg")
result = cv2.imread("panaroma/Panorama_image"+str(mid_image-1)+str(mid_image-2)+".jpg")
for i in range(mid_image):
    print("i : ", i)
    (r, matched_points) = panaroma.image_stitch([result, images[mid_image-i-3]], match_status=True)
    cv2.imwrite("panaroma/Panorama_image"+str(mid_image-i-2)+str((mid_image-i-3))+".jpg",r)
    crop("panaroma/Panorama_image"+str(mid_image-i-2)+str(mid_image-i-3)+".jpg")
    cv2.imread("panaroma/Panorama_image"+str(mid_image-i-2)+str(mid_image-i-3)+".jpg")
    result = cv2.imread("panaroma/Panorama_image"+str(mid_image-i-2)+str(mid_image-i-3)+".jpg")
(result1, matched_points) = panaroma.image_stitch([result1, result], match_status=True)
cv2.imwrite("panaroma/Panorama_image.jpg",result1)
crop("panaroma/Panorama_image.jpg")
cv2.imread("panaroma/Panorama_image.jpg")'''

    
while(total_iterations>=2):
    total_iterations = total_iterations / m
    images1 = []
    for i in range(int(total_iterations)):
        (result) = panaroma.image_stitch([images[i*2], images[(i*2)+1]], match_status=True)
        cv2.imwrite("panaroma/Panorama_image111"+str(int(total_iterations))+"_"+str(i*2)+str((i*2)+1)+".jpg",result)
        cv2.imwrite("panaroma/Panorama_image"+str(int(total_iterations))+"_"+str(i*2)+str((i*2)+1)+".jpg",result)
        crop("panaroma/Panorama_image"+str(int(total_iterations))+"_"+str(i*2)+str((i*2)+1)+".jpg")
        images1.append(cv2.imread("panaroma/Panorama_image"+str(int(total_iterations))+"_"+str(i*2)+str((i*2)+1)+".jpg"))
    images = []
    images = images1.copy()
print("end")
'''
if(total_iterations%2)==0:
    mid_image = total_iterations/2
else:
    mid_image = (total_iterations+1)/2
print("mid image", mid_image)'''

    


# In[ ]:




