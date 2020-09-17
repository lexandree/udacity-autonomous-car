# **Behavioral Cloning** 

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* model.png Model Architecture
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* track1.mp4 video recording of vehicle driving autonomously 
* Behavioral_Cloning_v1.ipynb more advanced colab notebook for the latest tensorflow version
* writeup_report.md summarizing the results (this file)

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of a convolution neural network ResNet50 with the last resudual block removed. Data preprocessing: lambda layer performs normalization, cropping and resizing, therefore, preprocessing for the inference is not required. The input image size is 160x320x3.

#### 2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting. More data was generated due to the horizontal flipping of the image and the reuse of the left and right views with the corresponding angle correction.

The model was trained and validated on different data sets to ensure that the model was not overfitting. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually.

#### 4. Advanced models

The more advanced models tested in the colab enviroment. Some keywords of these experiments are: ImageDataGenerator, flow_from_dataframe, multi_output mode, subclassing, Telegram callback, EfficientNetB7, s. Behavioral_Cloning_v1.ipynb. 

But it is overkill for the passage of track 1. Further reduction in the number of trained parameters, reduction of blocks is possible. adding some images from track 2 leads to instability on track 1 but does not improve the first turn of track 2.

#### 5. Model performance
Below is the video that demonstrates the performance of the trained model using 
[Udacity's driving simulator](https://github.com/udacity/self-driving-car-sim).  
[![LINK TO YOUTUBE](youtube.png)](https://www.youtube.com/watch?v=GRP7gyGeuls)


**Model Architecture**
![](https://github.com/lexandree/udacity-autonomous-car/blob/master/project4/model.png)
