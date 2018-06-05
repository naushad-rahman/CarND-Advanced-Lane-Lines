# Project 04 - Advanced Lane Finding
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## README

The code I used for doing this project can be found in `project04.py` and `Project04.ipynb`. All the line numbers I refer to in this document is for `project04.py`. 

---
## Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.


The camera calibration code is done in the functions `calibrate_camera` and `camera_setup` which can be found in cell 2 and 3.  The calibration is performed using chessboard pattern images taken using the same camera as the project videos, such as the one shown below:

![chessboard pattern](./camera_cal/calibration1.jpg)

The chessboard pattern calibration images (in the `cal_images` folder) contain 9 and 6 corners in the horizontal and vertical directions, respectively (as shown above). First, a list of "object points", which are the (x, y, z) coordinates of these  chessboard corners in the real-world in 3D space, is compiled. The chessboard is assumed to be in the plane z=0, with the top-left corner at the origin. There is assumed to be unit spacing between the corners. These coordinates are stored in the array `objp`.

```python
  # cal_images contains names of calibration image files
  for fname in cal_images:
      img = cv2.imread(fname)
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

      ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)
      if ret == True:
          objpoints.append(objp)
          imgpoints.append(corners)

  ret, cam_mtx, cam_dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
```

For each calibration image, the image coordinates (x,y), of the chessboard corners are computed using the `cv2.findChessboardCorners` function. These are appended to the `imgpoints` array and the `objp` array is appended to the `objpoints` array.

The two accumulated lists, `imgpoints` and `objpoints` are then passed into `cv2.calibrateCamera` to obtain the camera calibration and distortion coefficients as shown in the above code block. The input image is then undistorted (later in the image processing pipeline on line 409) using these coefficients and the `cv2.undistort` function.

## Pipeline (single images)

### 1. Example of Distortion corrected image

The image from the camera is undistorted using the camera calibration matrix and distortion coefficients computed in the previous step. This is done using the `cv2.undistort` function as shown below:

`undist = cv2.undistort(image, cam_mtx, cam_dist, None, cam_mtx)`

An example of an image before and after the distortion correction procedure is shown below.

![Example of Distortion corrected image][image1]

### 2. Thresholding

I have attempt to use different combinations of color and gradient thresholds and I have found that none of the combination is robust under varying lighting and contrast condition .After reviewing some literature , I have found that 2nd derivative operation (Laplacian) might be more suited to this purpose.By using a Laplacian filter (using `cv2.Laplacian`) on the image followed by thresholding it to highlight only the negative values (denoting a dark-bright-dark edge) it was possible to reject many of the false positives [ [Ref](http://www.eng.utah.edu/~hamburge/Road_Marking_Features_and_Processing_Steps.pdf) ]. The Laplacian resulted in better results than using combinations of Sobel gradients.

The thresholding operations used to detect edges in the images can be found in cell 8  `project_04.py` in the function called `find_edges`. We have obtained thresholded binary mask from the Laplacian operation . Thresholding in first done in s-channel of HLS colorspace and next thresholding in done in S-channel but simple threshold operation is used . Finally we have combined both the images as shown below 

`combined_mask = gray_binary AND (mask_one OR mask_two)`

The results obtained  shown below:

![Thresholding Example][image3]

### 3. Perspective transform

The perspective transformation is computed using the functions `get_perspective_transform` and `find_perspective_points` and it is  Code in cell number 6 and &  `project04.py`. `find_perspective_points` uses the method from Hough Transformation. Since the lanes are approximated as lines, it can be used to extract four points that are actually on the road which can then be used as the "source" points for a perspective transform.

Here is a brief description of how ths works:

1. Perform thresholding/edge detection on the input image
2. Mask out the upper 60% of pixels to remove any distracting features
3. Use Hough transforms to detect the left and right lane markers
4. Find the apex-point where the two lines intersect. Choose a point a little below that point to form a trapezoid with four points -- two base points of lane markers + upper points of trapezoid
5. Pass these points along with a hardcoded set of destination points to `cv2.getPerspectiveTransform` to compute the perspective transformation matrix

*Note: In case the hough transform fails to detect the lane markers, a hardcoded set of source points are used*

The original and warped images along with the source points (computed dynamically) and destination points used to computed the perspective transform, are shown below:

![Perspective Transform Example][image4]

### 4. Lane Detection

The lane detection was primarily performed using two methods -- 
1) histogram method 
2) masking method. 
The latter only works when we have some prior knowledge about the location of the lane lines. 

#### (a) Histogram Method

The first step in this method is to compute the base points of the lanes. This is done in the function `histogram_base_points`  in cell 15 `project04.py`. The first step is to compute a histogram of the lower half of the thresholded image. The histogram corresponding to the thresholded, warped image in the previous section is shown below:

![Histogram Plot][image5]

The `find_peaks_cwt` function from the `scipy.signal` is used to identify the peaks in the histogram. The indices thus obtained are further filtered to reject any below a certain minimum value as well as any peaks very close to the edges of the image. For the histogram shown above, the base points for the lanes are found to be at the points `297` and `1000`.

Once the base points are found, a sliding window method is used to extract the lane pixels. This can be seen in the `sliding_window` function in lines 308-346. The algorithm splits the image into a number of horizontal bands (10 by default). Starting at the lowest band, a window of a fixed width (20% of image width) centered at both base points is considered. The x and y coordinates of all the nonzero pixels in these windows are compiled into into separate lists. The base point for the next band is assumed to be the column with the maximum number of pixels in the current band. After all the points are accumulated, the function `reject_outliers` is used to remove any points whose x or y coordinates are outside of two standard deviations from the mean value. This helps remove irrelevant pixels from the data.

The filtered pixels are then passed into the `add_lane_pixels` method of the `Lane` class defined in lines 200-248. These pixels, along with a weighted average of prior lane pixels are used with `np.polyfit` to compute a second order polynomial that fits the points.

The polynomial is then used to create an image mask that describes a region of interest which is then used by the masking method in upcoming frames.



### 5. Radius of curvature and vehicle position

The radius of curvature is computed in the function named`compute_rad_curv` and code in cell 12. The pixel values of the lane are scaled into meters using the scaling factors defined as follows:
```python
ym_per_pix = 30/720 # meters per pixel in y dimension
xm_per_pix = 3.7/700 # meteres per pixel in x dimension
```
These values are then used to compute the polynomial coefficients in meters and then the formula given in the lectures is used to compute the radius of curvature.

The position of the vehicle is computed by the code in lines 455-456. The camera is assumed to be centered in the vehicle and checks how far the midpoint of the two lanes is from the center of the image.

```python
middle = (left_fitx[-1] + right_fitx[-1])//2
veh_pos = image.shape[1]//2
dx = (veh_pos - middle)*xm_per_pix # Positive on right, Negative on left
```

## Summary and Result

### Images

The complete pipeline is defined in the function `process_image`  all these steps and then draws the lanes as well as the radius and position information on to the frame. The steps in the algorithm are:

**Distortion correction → Edge Detection → Perspective Transform → Lane Detection (using Histogram or Masking methods) → Sanity Check**

An example image that was run through the pipeline is shown below:
![Final Image][image7]


### Videos

**Project video output**

The video is:  [project_video_out.mp4](./output/project_video_out.mp4)

**Challenge video output**

[Challenge video output](./output/challenge_video_out2.mp4)

The lanes in the harder challenge video were found to be very difficult to track with this pipeline. The algorithm has to be improved and made more robust for that video.

---
## Discussion

The pipeline was able to detect and track the lanes reliably in  the project video. With some tweaks (reversing the warping/edge detection), it also worked well for the challenge video. The main issue with the challenge video was lack of contrast and false lines.



### (c) Steerable filters

Steerable filters are convolution kernels that can detect edges oriented at certain angles. This might especially be useful in cases like the harder challenge video where the lane line is practically horizontal in some frames.

### (d) Different types of curve fits

Euler spirals, also known as clothoids, are parametric curves whose curvature changes linearly with the independent variable. They are frequently used in highway engineering to design connecting roads, on and off ramps etc. These might be a better candidate curve to fit to the lane pixels rather than simple second order polynomials.


[image1]: ./figures/fig1_undistort.png "Original and Undistorted images"
[image2]: ./figures/fig2_warp_options.png "Warp options"
[image3]: ./figures/fig3_edges.png "Thresholded Image"
[image4]: ./figures/fig4_perspective.png "Warp Example"
[image5]: ./figures/fig5_hist.png "Histogram"
[image6]: ./figures/fig6_masks.png "Lane masks"
[image7]: ./figures/fig7_result.png "Output"
[video1]: ./project_video_out.mp4 "Project video output"
[video2]: ./challenge_video_out2.mp4 "Challenge video output"
