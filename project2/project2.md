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

# Camera Calibration

Camera calibration task is an easy step and comes from project 1. The difference is the size of the chessboard (9x6).
calibration2.jpg is a suitable picture for our purpose:
![](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/calibration2.jpg)

The task consists from two steps: findChessboardCorners() and calibrateCamera().
The code for this steps can be found in [utils](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/utils/calibrate.py)

The function undistort() uses camera matrix and the distortion coefficients to undistort video frames.

# Create a thresholded binary image
The pipeline is prety simple:
  1. Convert to HLS color space and separate the V channel
  2. Take the derivative of l-channel in x
  3. Take the derivative of red-channel in x (color is a hyperparameter)
  4. Combine both with s-channel
  
All used thresholds are tunable. A modified GUI tool is used
https://github.com/lexandree/opencv-edge-finder-gui

# Perspective Transform
I have used hough lines from the project 1. Draw two Hough lines first, take 4 endpoints as sourcepoints for the getPerspectiveTransform(). Again, GUI tool is used.

# Detect Lane Pixels
Sliding window approach is used for the starting point. Detected pixels are averaged over a polynomial line
Further frames are searched within margins over this polynomial line.

# Determine the curvature of the lane and vehicle position
Polynomial line with the meter per pixel parameter allows us to estimate the lane radius. To estimate the transverse position, the X value for the lower point of the polynomial curve is taken.

# Inverse transform
The averaged polynomials are drawn over the original frame after inverse transform. This can be made with the same matrix:

`warpPerspective(..., flags=cv2.WARP_INVERSE_MAP)`

# Overlay picture
An example frame:

![](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/example_s.jpg)

[Jupyter notebook ](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/P2.ipynb) needs three steps:
  1. Run cells till "Load distortion matrix and transform matrix into the cloned project now!"
  2. Load [distortion matrix](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/undist.p) and [transform matrix](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/m_trans.p) into the cloned project now
  3. Continue with the rest
  
[Result video](https://github.com/lexandree/udacity-autonomous-car/blob/master/project2/project_video.mp4) saved then in the test_videos_output folder. The python notebook is created in the Google Colab.
