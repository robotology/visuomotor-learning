# visuomotor-learning
Repository for learning visuo-motor mapping and visual-tactile-proprioceptive integration in iCub and iCub_SIM

## How to run
- Run `yarpserver`.
- In `yarpmanager` open and run application file [icubSimStartup](https://github.com/robotology/icub-main/blob/master/app/iCubStartup/scripts/icubSimStartup.xml.template) shipped with `icub-main`. Make sure all modules can run without errors.
- In `yarpmanager` and open the application file [multisensory_integration_icubSim.xml](https://github.com/robotology/visuomotor-learning/blob/master/app/scripts/multisensory_integration_icubSim.xml).
- In `multisensory_integration_icubSim` tab, run all modules except `iCub_SIM`, `iCubGui` and `yarpmotorgui` (unless you know and need them). Connect all
- Open a terminal and type 
	```
	yarp rpc /motorBabbling/rpc
	```
to connect to `motorBabbling` command service. There, with `help` you can explore the possibility to control the babbling behaviours of iCub/iCub_SIM.
- To babbling with object, for example, using:
	```
	auto_obj_babble_arm right 2 3 
	#right for the right-arm, 2 for 2 different objects, and 3 for 3 repeats for each object
	```
	
- Abstracted skin events can be read from this port `/skinEventsAggregator/skin_events_aggreg:o`. Output is in following format:
```
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0 1.0 0.0 0.0 1.0 () (5 r_forearm 291 303 351)
```
or
```
0.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 (4 r_hand 101) ()
0.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 (4 r_hand 118) ()
```
with the integer number in `( )` represents the taxel number, e.g. `101`, `118` or `291`, `303`, in the corresponding body part, e.g. `r_hand` or `r_forearm`. The first table of 33 columns (for 33 taxel in simulation) uses `1.0` to represent the activated taxel. 

### The order of taxel are as follows:

1. Forearm  
- Lower patch (16 taxels): 3 15 27 39 51 63 75 87 99 111 123 135 147 159 171 183   
- Upper patch   
	- (7 taxels - icubSim) : 207 255 291 303 315 339 351  
	- (8 taxels - icub)    : 195 207 231 267 291 303 339 351  
2. Hand (the first 5 taxels are of fingers):  
- Left: 3 15 27 39 51 99 101 109 122 134  
- Right: 3 15 27 39 51 101 103 118 137 124  

## Some known bugs:
- `iKinGazeCtrl` can hang and cause the `motorBablling` to fail: `killall -9 iKinGazeCtrl` and restart `motorBabbling`  
- `iCub_SIM` may collapse after long run: stop and restart all modules  

## Related publications
```
@INPROCEEDINGS{8850681,
author={P. D. H. {Nguyen} and M. {Hoffmann} and U. {Pattacini} and G. {Metta}},
booktitle={2019 Joint IEEE 9th International Conference on Development and Learning and Epigenetic Robotics (ICDL-EpiRob)},
title={Reaching development through visuo-proprioceptive-tactile integration on a humanoid robot - a deep learning approach},
year={2019},
volume={},
number={},
pages={163-170},
keywords={biomechanics;haptic interfaces;humanoid robots;learning (artificial intelligence);manipulators;mechanoception;neural nets;robot vision;tactile sensors;touch (physiological);head joints;arm configuration;motor babbling;deep neural network;arm joints;tactile sensors;successful contacts;simulated iCub humanoid robot;reaching movement;visually elicited reaching;deep learning approach;visuo-proprioceptive-tactile integration;reaching development;tactile activations;tactile activation;successful reach;arm configurations;Robot kinematics;Head;Skin;Manipulators;Training;Robot sensing systems},
doi={10.1109/DEVLRN.2019.8850681},
ISSN={2161-9484},
month={Aug},}
```
```
@INPROCEEDINGS{8594519,
author={P. D. H. {Nguyen} and T. {Fischer} and H. J. {Chang} and U. {Pattacini} and G. {Metta} and Y. {Demiris}},
booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
title={Transferring Visuomotor Learning from Simulation to the Real World for Robotics Manipulation Tasks},
year={2018},
volume={},
number={},
pages={6667-6674},
keywords={calibration;humanoid robots;learning (artificial intelligence);manipulators;motion control;neural nets;robot vision;stereo image processing;robotic manipulation tasks;physical iCub robot;joint measurements;systematic error;accurate joint estimates;visuomotor predictor;image-to-image translation approach;physical robot;sensing error;unavoidable sources;underlying head configuration;stereo image pair;visuomotor deep neural network predictor;hand-eye coordination task;iCub humanoid;complex robots;accurate hand-eye coordination;visuomotor learning;Robot kinematics;Head;Task analysis;Robot sensing systems;Visualization;Manipulators},
doi={10.1109/IROS.2018.8594519},
ISSN={2153-0858},
month={Oct},}
```

