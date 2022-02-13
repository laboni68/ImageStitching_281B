# ImageStitching_281B
## How to run and compile

## Implemented Core Functions
`imageStitching`
- Taking the images to be stitched as the argument, this function calls the required functions to generate the final stitched image

<br />

`calculateHomography`
- Takes correspondences in between two images to calculate the homography matrix. After creating a matrix with those points, numpy linear algebra svd is used to generate the u, v, s.
<br />

`ransac`
- Randomly choosing 4 points out of the valid matched keypoints in the images, homography is calculated. Then finally it checks the `geometricDistance` to find whether they are inliers or outliers. The whole process will run until we find enough maximum number of inliers (which is defined using a definite threshold, in my case which is 4.0)
<br />

`warpImage`
- Takes the image required to be warped with respect to the base image as parameter
- Takes the new image width, height and homography matrix as parameter
- Calculates the transformed image corners 
- Then interpolate the pixel values from the original image

<br />

`Detect_Feature_And_KeyPoints`
- `cv2.xfeatures2d.SIFT_create()` is used to get the descriptors of the images.
- Then `detectAndCompute()` is used to get the keypoints and features of the iamges.
<br/>

`matchKeypoints`
- Using those keypoints, this function finds the all possible matches.
- Then using all possible matches, find only the valid matches using `All_validmatches()` using `lowe ratio`
- Finally, if the valid mathces is greater than 4 (not possible to find homography if it is less than 4), then it computes the homography matrix using `ransac` function.

<br/>

`Crop`
- Cropping the extra black part of the image.

<br/>

`find_mid`
- To find the middle image to take that image as the base image for left and right hand stitching.


## Files
`Image_Stitching_final.py`
- All the codes for stitching images are there

`Image_Stitching_final.ipynb`
- Same as python file but can be run in jupyter notebook directly to get the desired output

## Folders
`data`
- Contains a set of 21 image files which need to be stitched together.
<br />

`Intersection`
- Contains a simple set of 8 images which we want to stitch together.
<br />

`panaroma`
- An empty folder, the generated images from the algorithm will be saved here.

## Things to look out
`Dataset`
- The convention I have assumed is that the images are named as '0.jpg', '1.jpg' and so on.
- Sometimes due to the MAC OS, .DS_store is created in the image folder. I have deleted them while counting the number of images present in a folder. So, if there is anything other than the image files, it may get errors while compiling.

<br />

`References`
- https://kushalvyas.github.io/stitching.html
- https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87
- https://medium.com/swlh/image-processing-with-python-image-warping-using-homography-matrix-22096734f09a
- https://github.com/avinashk442/Panoramic-Image-Stitching-using-invariant-features/blob/master/panorama.py
- https://github.com/hughesj919/HomographyEstimation




