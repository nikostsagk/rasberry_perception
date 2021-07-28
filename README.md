# rasberry_perception

![strawberry_localisation](https://user-images.githubusercontent.com/16948324/76231446-2c98b380-621d-11ea-8624-8e472c2f08f8.gif)

The rasberry_perception package aims to interleave ROS and deep learning frameworks for perception. If using any of the models in research please contact [Raymond Kirk](https://github.com/RaymondKirk) to obtain the relevant citation and ensure no conflict of interest.

## Quick start

```bash
roslaunch rasberry_perception detector.launch backend:="detectron2" password:="obtain_from_raymond" image_ns:="/your_camera/colour" depth_ns:="/your_camera/depth" score:="0.5"
```

## Installation

[Cuda 10.2](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=deblocal) must be installed locally to run gpu based backends. 

```bash
cd catkin_ws/src
git clone https://github.com/RaymondKirk/rasberry_perception
catkin build rasberry_perception
```

## Detection Backends

Modular detection backends are available in `rasberry_perception` enabling users to utilise deep learning 
frameworks/non-ros methods to detect objects. 

You can try to launch both the backend and detector with the command below:

```bash
# Run together (will download the backend from docker_hub if it exists)
roslaunch rasberry_perception detector.launch colour_ns:="" depth_ns:="" score:="" show_vis:="" backend:="" backend_arg1:=""

# Or run separately! (Will use a local installation of the backend if available)
rosrun rasberry_perception detection_server.py backend:="" backend_arg1:=""
roslaunch rasberry_perception detector.launch colour_ns:='' depth_ns:='' score:=''
```

### Adding a new detection backend 

Adding custom backends such as TensorFlow, PyTorch, Detectron, Onnx etc. to `rasberry_perception` is easy. 
See [interfaces](src/rasberry_perception/detection/interfaces/) for examples.

A simple example given in four steps, register the name in the detection registry with the class decorator (1), inherit from the 
base (2), implement the service call logic (3) and finally add to the `__all__` definition 
[here](src/rasberry_perception/detection/interfaces/__init__.py) (4). 


```python
import ros_numpy
from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.msg import Detections, ServiceStatus

@DETECTION_REGISTRY.register_detection_backend("CustomBackendName")  # (1)
class CustomVisionBackend(BaseDetectionServer):  # (2)
    # These args are passed from ros parameters when running the backend
    def __init__(self, custom_arg1, custom_arg2, default_arg1="hello"): 
        # Do your imports here i.e import image_to_results_function
        # Do initialisation code here
        self.busy = False 
        BaseDetectionServer.__init__(self)  # Spins the server and waits for requests!

    def get_detector_results(self, request):  # (3)
        if self.busy:  # Example of other status responses
            return GetDetectorResultsResponse(status=ServiceStatus(BUSY=True))
        # Populate a detections message
        detections = Detections()
        # i.e. detections = image_to_results_function(image=ros_numpy.numpify(request.image))
        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections)
```

When launching the detection server via `rosrun` or `roslaunch` you can pass in arguments to your custom backend as you 
would usually. The node will fail if you do not pass any non-default arguments such as `custom_arg1` and `custom_arg2` 
in the example.

```bash
rosrun rasberry_perception detection_server.py  backend:="CustomBackendName" _custom_arg1:="a1" _custom_arg2:="a2" _default_arg1"="world"
```

## Git strategy
Based off [this](https://docs.google.com/presentation/d/1UDsfChvRGO-f4m43OagRsdkWgTNlWnBxtvdC2COKL-k/edit#slide=id.p4)

* Fork saga repo https://github.com/SAGARobotics/rasberry_perception
* To add a feature create a descriptive branch name, and link it to an issue #
    - Branch should be in your own fork unless working with someone else
* When the feature is finished or ready to be picked up by someone else, test it on your machine (basic test strategy)
* Create a pull request from your branch to SAGARobotics develop_pc branch, assign PR to someone
* Once the pull request has been tested by someone else it can be merged (basic test strategy)
    - Delete branch when merged
* If it has been tested on the Jetson already the develop_pc branch can be merged with develop_jetson. Otherwise repeat tests on Jetson and merge to develop_jetson
* Once a month the system is tested extensively and the develop branch is merged into master and a release branch is made (extensive test strategy)

## Testing Strategies
### Basic Testing Strategy
1. Run gripper_perception and robot_perception simultaneously on [rosbag](https://drive.google.com/file/d/1Hou6I4-i5ziNpsMPYu96Gs7I6dXLnDGk/view?usp=sharing)
   * Note FPS
    * Turn off gripper_perception and note FPS
    * Turn off robot_perception and note FPS
2. Test tracking
    * Sense check with RQT
   * TODO automate
    
3. Test segmentation
    * sense check with RQT
    * TODO automate
    
   
### Extensive Testing Strategy
4. Test berry localisation
    * Sense check with Rviz on testbench
5. As above but with different real scenarios in a polytunnel
