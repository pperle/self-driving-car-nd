# Finding Lane Lines on the Road

#### The goals / steps of this project are the following:

* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report

[//]: # (Image References)
[image1]: ./test_images/solidWhiteRight.jpg "Original"
[image2]: ./test_images_output/solidWhiteRight_01_Grayscale.jpg "Grayscale"
[image3]: ./test_images_output/solidWhiteRight_02_BlurredGray.jpg "Blurred"
[image4]: ./test_images_output/solidWhiteRight_03_Edges.jpg "Edges"
[image5]: ./test_images_output/solidWhiteRight_04_MaskedEdges.jpg "Masked"
[image6]: ./test_images_output/solidWhiteRight_05_Final.jpg "Final"

---

## Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps.

First, I converted the images to grayscale.
![Grayscale][image2]

Then I applied a Gaussian blur with a 5x5 kernel.
![Blurred][image3]

In order to find edges in the image I used the Canny Edge Detection algorithm as described in Lesson 2, with a 1:3 threshold (50:150).
![Edges][image4]

Because the front facing camera is mounted in a fixed position it is save to apply a mask as the lane lines will always be in the same region of the image.
![Masked][image5]

The Hough line transformation is used to detect lines from the remaining edges in the masked image.
![Final][image6]


In order to draw a single line on the left and right lanes, I modified the draw_lines() function by calculating the gradient of each line and grouping the lines by positive or negative gradient. All gradients below +/-15° are ignored. After calculating the average for gradient and the average intersection with the y-axis for each group I was able to calculate two new lines from these values.


###2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be what would happen when the driver is switching lanes, because the perspective relative to the street would change.

Another shortcoming could be a very curvy street, as seen in the challenge video the lane recognition does not work as good with curves as in a atraigh outline. This is expected because the pipeline only draws one straight line.


###3. Suggest possible improvements to your pipeline

A possible improvement would be to switch from drawing one straight line to maybe draw multiple lines depending how curvy the street ahead is.