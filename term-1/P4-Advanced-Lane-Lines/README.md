# Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./examples/undistort_output.png "Undistorted"
[image2]: ./examples/road_undistort.png "Road Transformed"
[image3]: ./examples/road_binary_combo.png "Binary Example"
[image4]: ./examples/road_warped.png "Warp Example"
[image_bw]: ./examples/road_binary_combo_warped.png "Binary Warp Example"
[image5]: ./examples/road_color_fit_lines.png "Fit Visual"
[image5-2]: ./examples/road_color_fit_lines-2.png "Fit Visual"
[image6]: ./examples/output.png "Output"
[video1]: ./project_video.mp4 "Video"

---
### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first to forth code cell of the juypter notebook located in "./AdvancedLaneFinding.ipynb".  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps in the 7th code cell).  Here's an example of my output for this step.


![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warp()`, which appears in the 3rd code cell of the juypter notebook.  The `warp()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
max_y = cv2.imread(test_images[0]).shape[0]
src = np.float32([[230, 700], [580, 460], [700, 460], [1060, 690]])
dest = np.float32([[230, max_y], [230, 0], [1060, 0], [1060, max_y]])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 230, 700     | 320, 720        | 
| 580, 460      | 230, 0      |
| 700, 460     | 1060, 0      |
| 1060, 690      | 1060, 720        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I combined the two previews steps and fit my lane lines with a 2nd order polynomial (15rd & 17th code cell of the juypter notebook).

![alt text][image_bw]
![alt text][image5]

If there has already been an image analysed it is unecessary to do all the expensive calculation using "sliding windows" and instead you can just search in a margin around the previous line position.

![alt text][image5-2]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The curvature and the center offset have been caluclated in the 19th & 21st code cell of the juypter notebook

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

The `video_pipeline(image)` can be found in the 23rd code cell of the juypter notebook.

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video_detected.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The hardest part of this pipeline was trying to find good values for the edge detection part. I also had problems understanding how to calculate the curvature.
As I didn't introduce any smoothing (e.g. gaussian blur) the pipeline will have problems when there are a lot of edges of the road (e.g. after construction works).
