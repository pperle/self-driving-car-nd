# Vehicle Detection Project

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image_HOG_1]: ./output_images/car_images.png
[image_HOG_2]: ./output_images/noncar_images.png
[image_HOG_3]: ./output_images/hog_car.png
[image_HOG_4]: ./output_images/hog_noncar.png
[sliding_1]: ./output_images/sliding_01.png
[sliding_2]: ./output_images/sliding_02.png
[sliding_3]: ./output_images/sliding_03.png
[sliding_4]: ./output_images/sliding_04.png

[heatmap_1]: ./output_images/sliding_04.png
[heatmap_2]: ./output_images/heatmap_01.png
[heatmap_3]: ./output_images/heatmap_02.png
[heatmap_4]: ./output_images/heatmap_03.png

[image3]: ./examples/sliding_windows.jpg
[image4]: ./examples/sliding_window.jpg
[image5]: ./examples/bboxes_and_heat.png
[image6]: ./examples/labels_map.png
[image7]: ./examples/output_bboxes.png
[video1]: ./project_video.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points

---

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the forth code cell of the IPython notebook.  

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image_HOG_1]
![alt text][image_HOG_2]

Here is an example using the `YCrCb` color space and HOG parameters of `orientations=8`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)`:

![alt text][image_HOG_3]
![alt text][image_HOG_4]

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and came to the colution that the paramates shown in the examples worked best for me.

#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM in the seventh code cell of the IPython notebook with a test accuracy of 0.9856.

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

The code for this step is contained in the 10th code cell of the IPython notebook. I have reduced the search area (only y-axis ([400,650], [400,550], [400,520])) and the windows ((128, 128), (96, 96), (80, 80)) with each integration .

![alt text][sliding_1]
![alt text][sliding_2]
![alt text][sliding_3]



#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on three scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.  Here are some example images:

![alt text][sliding_4]
---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video_result.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

##### simple sliding window search
![alt text][heatmap_1]

##### heatmap of the boxes with a threshold of 1
![alt text][heatmap_2]


##### result of `scipy.ndimage.measurements.label()`
![alt text][heatmap_3]

##### endresult
![alt text][heatmap_4]



---

###  1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

1. The pipline might fail when cars are facing the camera as the model was only trained on cars driving in front of the camera.
2. Adding more data like the KITTI vision benchmark suite or the udacity data set could improve results.
3. `skimage.feature.hog()` seems the be the bottle neck of the pipline.
4. A deep learning approve might be better (YOLO or SSD).


---

### Resubmission

For the resubmission I utilizing the fact that in the subsequent frames the cars are located at or near the same positions and implemented a multi-frame accumulated heatmap (20th code cell of the IPython notebook).
