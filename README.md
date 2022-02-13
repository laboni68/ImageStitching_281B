# ImageStitching_281B
## How to run and compile

## Implemented Core Functions
`image stitching`
- Taking the parameter of the images to be stitched this function calls the required function to generate the final stitched image

<br />

`calculateHomography`
- Takes correspondences in between two images to calculate the homography matrix. After creating a matrix with those points numpy linear algebra avd is used to the u, v, s.
<br />

`ransac`
- Randomly choosing 4 points out of valid matched keypoints in the images homography is calculated. Then finally checking the geometric distance to find whether they are inliers or outliers. The whole process will run until we find enough maximum number of inliers (which is defined using a definite threshold in my case which is 4.0)
<br />

`warpImage`
- Takes the image needs to be warped with respect to the base image as parameter
- Takes new image width and height and homography matrix as parameer
- Calculates the transformed image corners 
- Then interpolate the pixel values from the original image

<br />

`Detect_Feature_And_KeyPoints`
- `cv2.xfeatures2d.SIFT_create()` is used to get the descriptors of the images.
- Then `detectAndCompute()` is used get the keypoints and features of the iamges.
<br/>

`matchKeypoints`
- Using those keypoints, this function finds the all possible match.
- Then using all possible match find only the valid matches using `All_validmatches()` using `lowe ratio`
- Finally if the valid mathces is greater than 4 (not possible to find homography if it is less than 4) then it computers the homography m atrix using `ransac` function.

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
- Contains a set of 21 images files which need to be stitched together.
<br />

`Intersection`
- Contains a simple set of 8 images which we want to stitch together.
<br />

`panaroma`
- An empty folder, the generated images from the algorithm will be saved here.

## Things to look out
`Dataset`
- The convention I have assumed is that the images are named as '0.jpg', '1.jpg' and so on.
- Sometimes due to the MAC OS, .DS_store is created in the image folder. I have deleted them while counting the number of images present in a folder. So, if there is anything other than the image files, it may get error while compiling.


