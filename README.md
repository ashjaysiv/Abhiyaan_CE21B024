# Abhiyaan_CE21B024

# Outputs

https://drive.google.com/drive/folders/10dv1gb62qq8xgqrp-1gf5T-JX_g1Mhbs?usp=share_link

Resources:

This contains all the outputs for the tasks.

## Name:
Ashmitha Jaysi Sivakumar
## Roll no:
CE21B024
##  Previous Experience
* CFI Analytics Club Project Member in the project AI on EDGE. We built a visual question answering model and hosted it on a webserver. Therefore, in the process of building the model with the team, I learnt about various CNNs, NLP, transformers, LSTMs, Attention models and other stuff. 
* Techsoc BuildSchool 2.0: Me and my team are a part of the 10 teams which got selected. We are building a personalised email for insti. Hence, I have an average level experience with webdev as well.
## Current PORS:
* Saathi Mentor
* Manager at E-Cell IITM (Editorial and Research)
## Why I want to work in the team: 
* I know I love to code but just coding with no purpose is not fun. I would like to see my code come to life. Not as a meow of a cat, but as a roar of a lion. Hence I see Ahiyan as a means to give my code its place in the world.
* Getting into IITM, it is necessary to utilize all the opportunities it gives. Otherwise what's the difference between a normal college and an IIT. Hence being in a competitive team is one of those things in my bucket list for college life.
* I want to work on something Day and Night. I want to feel that adrenaline. I want to get that immense satisfaction after a day's work.

## Relevant Courses:
### In Institute:
* CS1100: Introduction to Programming 1
* CE3025: Traffic Engineering
### Online:
Supervised Machine Learning: Advanced Learning Algorithms-Andrew Ng
Supervised MAchine Learning: Regression and Classification- Andrew Ng
edX Verified Certificate for CS50’s Introduction to Computer Science- edX



## Task 1 - Get Set …
* Installed Ubuntu 20.04 on a virtual machine
* Installed ROS Noetic

## Task 2 - Go!
### Prerequisites:
* Went through the provided ROS tutorials and turtlesim tutorials
### Subtask 1:
* Wrote a node that publishes the message: ‘NO.1 CFI TEAM’ to a rostopic named
‘/chatter’
* created a subscriber node that subscribes to this topic and prints this
message.

### Subtask 2:
* altered the subscriber node such that it adds the text ‘ABHIYAAN’ to the received message.
* new message is published to a new topic ‘/absolute_truth’
* **NOTE**: nodes have been written in c++



### Subtask 3:
The terminal commands used are:
* roslaunch trurtlesim turtlesim_node

* rosservice call /kill “turtle1”

* rosservice call /spawn 1 5 1.5707 “turtle1”
* rosservice call /spawn 5 1 1.5707 “turtle2” (don't remember what were the exact coordinates for the corresponding picture, so a random coordinate has been entered)



## Task 3 - Hunter x Hunter
### Subtask 1 - Turtle Xenzia
#### Approach 1: 
* Made the turtle move closer to the coordinate of the spawned turtle by altering the linear velocity and the angular velocity
* A problem which occurred was that, once the turtle bumped into the wall, it just stayed there. It would just keep slamming into the wall.

#### Approach 2:
* Now what was done is that whenever the turtle hit the wall, it was made to bounce back
* Killing the turtle if it hits the wall is next 

### Subtask 2 - PacTurtle

## Task 4 - Red Light Green Light:
### References:
https://github.com/techwithtim/Pygame-Car-Racer/blob/main/tutorial4-code/main.py

### Part 1:
WRITE BEHAVIOR PLANNER (flow chart)
Make it turn properly
Stop at the lights
Slow down at yellow
Get the trajectory with points
* installed Pygame and the given files

#### Approach 1:
Make a single car follow the traffic conditions (red and green)  in a specific path
* Step 1: made the functions for the car to rotate and move 
* Step 2: obtained a trajectory for the car
* Step 3: made the car follow the path (without considering the traffic signs)
* Step 4: analyzed the traffic behavior on a roundabout and thus obtained the pixel positions for where the car would stop and under what conditions.
//insert image 
* Step 5: made functions for each signal and applied the conditions for the stopping
* Step 6: hence the single car obeyed the traffic signals

#### Improvements to make: 
* Car touches the lane a little bit. Can do the path corrections at the end.
* Need to make it slow down/ stop on a yellow 

#### Approach 2:
Make multiple cars follow the traffic conditions (red, yellow and green) in a specific path.
* Step 1: generate multiple cars
* Step 2: Make sure they stop with a safe distance between them.
* Step 3: Make sure no two cars collide with each other in their individual path. This is mostly taken care of by the signals. 

#### Possible Improvements:
* Make all paths that are possible 4C2
* Make the cars not touch the lane by adjusting the points chosen in the path.
* Make the speeds and traffic signals more realistic
* Put the vehicle spawns in a loop. 


## Task 5 - “I gave so many signs …”
Goal of the task is to make a model which can accurately detect traffic signs. Here is a summary of the used models and datasets.



Model: VGG19

Dataset: https://www.kaggle.com/datasets/andrewmvd/road-sign-detection(very small dataset)

Accuracy on Train: 90%


Model: VGG19

Dataset: , https://www.kaggle.com/datasets/andrewmvd/road-sign-detection, 

Accuracy on Train: 50.62%

Model: Resnet152

Dataset: https://www.kaggle.com/datasets/andrewmvd/road-sign-detection

Accuracy on Train: 48.21% (on 5 epochs can be further increased if colab runtime wasnt exceeded)

Other models which can be tried out are Yolo7 (which is expected to have the highest accuracy). The test data accuracy hasn't been calculated due to exhaustion of colab usage and lack of time. But the results were tested using one or two sample images, which did give satisfactory results for general images.



### Other Approaches:

For the allotted time the models were only possible to code and train. Further scope of improvement can be brought by  integrating the ideas postulated in the following research papers:
#### https://arxiv.org/pdf/2207.06067.pdf

This paper aims to tackle the problems caused by a small dataset and the unbalanced distribution of traffic signs by using PYRAMID TRANSFORMERS.

The proposed transformer inherits an intrinsic scale invariance inductive bias and is able to learn local feature representation for objects at various scales. Thus, enhancing the network robustness against the size discrepancy of traffic signs.

**It  achieves 77.8% mAP on GTSDB**
#### What is a Vision Transformer? How does it compare itself with CNN?
* utilizes a purely attention-based model
* Basic components: multi-head self attention, and feed forward neural network with normalization and residual shortcuts.
* fixed pixel patches are linearly projected along with its positional embedding the input vector is formed.

#### How is this better than a normal vision transformer?:
* Vision Transformers falls short as it learns the global interactions between the patch tokens and hence cant be extended to high resolution images. (memory complexity) 
* A pyramid like structure is introduced to generate multi-scale feature maps for dense prediction.
* There are two major blocks in the architecture:
PB: embed multi-scale context into tokens

Structure: 
Image → PRM (Pyramid Reduction Module) → MHSA (Multi-Head Attention) → PCM (Parallel Convolutional Modules)


NB: inject convolutional bias, divided into two parallel branches, which together model locality and long-range dependency, 
* Input:  class token from the third PB is concatenated with F from the third PB and then combined with positional embeddings 
* element-wise sum is then applied to the features from MHSA and P RM
FFN
* Similar Structure but no PRM.
Lr = 0.0001
Batch_size = 16
https://arxiv.org/ftp/arxiv/papers/2003/2003.03256.pdf


Yolo?



### Other Datasets:
http://www.nlpr.ia.ac.cn/pal/trafficdata/recognition.html

### Problems faced:
* Often distorted by camera motion, different adverse weather conditions, and poor lighting conditions that significantly increase the difficulty level of this task.
* Absence of a large Dataset

https://www.researchgate.net/publication/342987130_Difficulties_of_Traffic_Sign_Recognition


## Task 6 - "Hey! I'm walking here!"

https://colab.research.google.com/drive/1ftpSEIciZREoTeLO8_lrC3EsOPK-rEqj


Special Note: Task 6 has a log file containing the position, velocity and acceleration of the moving object.

### Single Pedestrian:

#### References:
https://pysource.com/2021/11/02/kalman-filter-predict-the-trajectory-of-an-object/

#### Approaches:
* Identified the tasks at hand
Identify the circle
Extract its position
Track its position 
Calculate the speed (distance / time)
Calculate the acceleration (v2 - v1/t)
(assumption is that between two frames the direction won't change much so the above is carried out)
Calculate the slope ( y2 - y1 / x2 - x1)

* KalmanFilter: used to predict the next path of a moving object 

htps://arxiv.org/ftp/arxiv/papers/1204/1204.0375.pdf#:~:text=A%20Kalman%20Filtering%20is%20carried,in%20wireless%20networks%20is%20given.



* For each frame, the ball is masked by giving the range of pixel values for the color of the ball. (Here blue)
* We then use cv2. HoughCircles  (on the processed frames) to identify the center and radius of the circle. 
https://www.youtube.com/watch?v=Ltqt24SQQoI

* With the obtained centers of each circle in frame, the further path of the ball can be hence predicted.
* Here is the path that was taken by the ball in the video
* Here is a log of all the positions, velocities, acceleration of each instant
* Here is the video with the predicted direction of the ball in the video

### Multiple Pedestrian:
## Task 7

* git clone {current repo}

* cd {current repo}

* git submodule add {submodule repo}

* git add .

* git commit -m “Add submodule”

* git push origin  





