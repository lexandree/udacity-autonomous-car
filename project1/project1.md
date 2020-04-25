# Project 1 - Finding Lane Lines on the Road

The goal of this first project was to create a simple pipeline to detect road lines in a frame taken from a roof-mounted camera.

I was very impressed how a simple GUI tool can help to find right parameters.
A modified version with hough lines window is created:
https://github.com/lexandree/opencv-edge-finder-gui
The first run on video clip shows that an approximation approach is necessary. The next and final version with the class showed its effectiveness on all video clips.

The jupyter notebook clones P1 template and creates all desired output images and video clips. It was run on Google Colab vm.

[P1 notebook](https://github.com/lexandree/udacity-autonomous-car/blob/master/project1/P1.ipynb)

I have six test images as an output with choosen parameters. These parameters are default parameters of the class constructor

![six test images](https://github.com/lexandree/udacity-autonomous-car/blob/master/project1/test_images_output/pict.png)

[The last output video clip challenge.mp4](https://github.com/lexandree/udacity-autonomous-car/blob/master/project1/test_videos_output/challenge_out.mp4)

**Pipeline.** 

The pipeline is prety simple:
  1. Covert to grayscale
  2. Apply gaussian blur
  3. Get edges with canny function
  4. Mask "cannied" image - set  the region from y=0 to height/2 to black or zero
  5. Draw lines with the hough function
  6. Select only those lines that have reasonable parameters - slope and intercept
  7. Compute mean slope and intercept, mix those with global means, draw two lines from y=height/2 till full height
        
This pipeline has a static parameters and can not find the lines ewerywhere. Under unfavorable starting conditions, the lines are determined incorrectly and the algorithm takes time to stabilize the line positions. It is easy to simulate taking an object already initialized on another video clip or image.
