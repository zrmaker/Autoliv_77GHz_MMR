[img]: ./data/snr.png

# Autoliv 77GHz Multi-Mode Radar (MMR) Driver
### &copy; Renyuan (Leo) Zhang and Fuheng Deng @ TuSimple

***
__Usage:__
```
roslaunch autoliv autoliv.launch
```

This driver obtains Autoliv MMR targets information from CAN bus. It consists of sending request messages, receving parser and visualization module. The updated documentation can be seen from [here](https://drive.google.com/file/d/1zfzqNDH9UyX53ZeJzj3LbW7oD1lHy9ad/view?usp=sharing). The evaluation report by Leo is shown [here](https://docs.google.com/document/d/1M_NcZc1VBaWdLAGSzmukq5m5ttpycX54jE2wl7v1Ylg/edit?usp=sharing).

__List of ROS message:__
```
/autoliv/diagnostics    # diagnostic information
/autoliv/markers        # markers for visualization
/autoliv/targets        # target information
/autoliv/versions       # versions of radar
```

__Example */autoliv/targets* message:__
```
header: 
  seq: 618120                   # sequence number
  stamp: 
    secs: 1521585931            # ROS (epoch) time
    nsecs: 536272379
  frame_id: "autoliv"
target_id: 34                   # target id defined by per radar measurement
x: -7.99607276917               # target lateral location related to radar in meters (left: negative, right: positive)
y: 18.7676010132                # target longitudinal location related to radar in meters (pointing front, only positive)
vx: -2.24595570564              # velocity of target projected to lateral axis in m/s (left: negative, right: positive)
vy: 5.27148771286               # velocity of target projected to longitudinal in m/s axis (towards: negative, away: positive)
range: 20.3999996185            # radar measure target range in meters
velocity: 5.73000001907         # velocity of target in m/s (towards: negative, away: positive)
bearing: -23.0767822266         # radar measure tareget bearing in degrees (left: negative, right: positive)
snr: 6.21557760239              # signal-to-noise ratio of the target
flags: 1                        # flag: 0 - short range radar; 1 - mid range radar; 2 - moving target indication
```

The SNR plot can be seen below:

![alt text][img]

The accurate max range is about 60 meters and the FOV is about 80°.

The filtering algorithm is implemented separately in another repo [radar-tool](https://github.com/TuSimple/radar-tool). 

__Full communication cycle:__

+ Send a sync message at 0x100.
+ Command all the sensors at 0x201 to 0x204.
+ Target messages are beginning at 0x300 and max of 140 dectections.
+ Diagnostic messages are 0x3E1 to 0x3E3. A diag code True indicates a serious fault.
+ Version message is at 0x3EF.
+ 0x3FF gives the valid detections.
