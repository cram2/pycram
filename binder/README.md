<!-----------------------TABLE OF CONTENTS---------------------->
<h1><summary>Table of Contents</summary></h1>

<h3>

1. [Binder](#binder)
2. [Project Integration](#ProjectIntegration)
    1. [Docker Foundation](#dockerbasics)

        1. [Dockerfile](#dockerfile)
        2. [docker-compose.yml](#compose)
        3. [entrypoint.sh](#entry)
    2. [webapp.json](#webapps)
        1. [Custom RvizWeb configuration](#rvizweb)
        2. [XPRA](#xpra)
  
3. [Tutorial](#tutorial)
   1. [Pick up object with Visualization](#pickup)
 
</h3>

<!------------------------Binder---------------------------->
<a name="binder">
<summary style="text-align: center;"><b><h1>
                      Binder 
</h1></b></summary>


> BinderHub is a kubernetes-based cloud service that allows users to share reproducible interactive computing environments from code repositories. It is the primary technology behind [mybinder.org](mybinder.org). 

On binder it is possible to build configured projects and run those in a virtual environment. One simple way to configure projects is with Docker.
However building projects might need a lot of resources depending on the projects size. This might lead to problems with the binder instance at [mybinder.org](mybinder.org) as the resources are limited to about 1 - 2 GB. The project [intel4coro](https://www.uni-bremen.de/zmml/projekte/intel4coro) runs a server at [binder.intel4coro](https://binder.intel4coro.de/) to launch projects that require more resources. Already built projects are saved and can be launched anytime without the need to build again. However when the project slightly changed (e.g. git commits) it will need to build again.



When a project is built, a link can be used to share this Hub in its default configuration:

 <p align="center">
  <img src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/Binder_default.png">
</p>


It is also possible to open a specific file by default when clicking on the link, which can be useful when sharing specific demos. To do this, right click the file that should open at startup and click <b> Copy Shareable Link </b>:

 <p align="center">
  <img src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/Binder_default.png">
</p>

<!--------------------Project Integration------------------------>
<a name="ProjectIntegration">
<summary style="text-align: center;"><b><h1>
                      Project Integration 
</h1></b></summary>

<a name="dockerbasics">
<summary style="text-align: center;"><b><h2> 
                      Docker Fundamentals
</h2></b></summary>
</br>


<a name="dockerfile">
<h3> Dockerfile </h3>

</br></br>
<summary>
1. Setup base system</summary>

The Dockerfile describes how the system will be set it up. A base system is defined at first to set the desired distribution as well as other predefined requirements. This is done with the following statement:

`FROM intel4coro/base-notebook:20.04-noetic-full-xpra` 

After that you can prepare the required dependencies for the specific Docker/Binder in this file. The following steps contain the Dockerfile of this repository as an example. If you want to build a Binderhub using pycram, you can just paste the code blocks for the wanted steps into your Dockerfile:

<br></br> 

<summary>
2. Setup for pycram</summary>
<br>
To setup a pycram workspace with docker it is neccessary to clone the respective repositories for that. For this simply create a rosinstall file to clone all at once. A reference for this can be this <a href="https://github.com/K3cks/pycram/blob/binder-example/binder/pycram-http.rosinstall">rosinstall</a> file. This  differs from the standard rosinstall for pycram since the intitialization of submoudles needs to be done seperately.
<br>

```
ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
COPY --chown=${NB_USER}:users . pycram/
RUN vcs import --input pycram/binder/pycram-http.rosinstall --recursive
```

<br>
<summary>
3. Clone pycram into workspace</summary>

```
RUN cd pycram \
  && git submodule update --init \
  && git clone https://github.com/Tigul/neem_interface_python.git src/neem_interface_python \
  && cd src/neem_interface_python \
  && git clone https://github.com/benjaminalt/neem-interface.git src/neem-interface

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/requirements.txt --user 
RUN pip install --requirement ${PYCRAM_WS}/src/pycram/src/neem_interface_python/requirements.txt --user \
  && pip cache purge
```

<br>
<summary>
4. Build pycram workspace</summary>

```
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN catkin build
```


<br>

<summary>
5. Start entrypoint.sh</summary>

```
WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz
```

</li>

<br><br> 

<a name="compose">
<li><b><h3>docker-compose.yml</h3></b>
<p>
To specify the the Docker image a compose file should be defined as a yml file. This will configure required capabilites such as the location of the Dockerfile and the entrypoint, permissions and the necessary drivers. This project used the following compose file, saved as <b>docker-compose.yml</b>:
<p>

```
version: '3'
services:
  pycram:
    image: pycram:binder-xpra
    build:
      context: ../
      dockerfile: ./binder/Dockerfile
    stdin_open: true
    tty: true
    ports: 
      - 8888:8888
    privileged: true
    # user: root
    command: jupyter lab --allow-root --NotebookApp.token='' --no-browser --ip=0.0.0.0
    entrypoint: ["/home/jovyan/work/binder/entrypoint.sh"]
    volumes:
      - ../:/home/jovyan/work
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - DISPLAY
      - QT_X11_NO_MITSHM=1
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```
</li>
<br><br>


<a name="entry">
<li><b><h3> entrypoint.sh </h3></b>
</a>

The entrypoint is usually executed when the built system should start. This will set some defaults as sourced workspaces or files launched by ros. An example for this would be the following code as a file called <b>entrypoint.sh</b>:

``` 
#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PYCRAM_WS}/src/pycram/binder/rvizweb_config.json &
roslaunch --wait pycram ik_and_description.launch &

cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json 

exec "$@"
``` 

In short this does the following instructions:
- source pycram workspace
- start ros-specifics such as
  - roscore
  - launch rvizweb with a custom config file
  - launch the ik solver for pycram
- copy webapps file and execute previous commands
</li>

<br>

<a name="webapps">
<summary style="text-align: center;"><b><h2>
                        Adding Webapps
</h2></b></summary>
<br>
This file defines the available apps for this Binderhub as a json file. The main parameters for the respective entries are:
<p>
 
- ```name```: Name of the App
- ```title```: Title of the App
- ```icon```: Link to a picture as svg
- ```url```: url to the index of the build folder
- ```start```: Defines if the app should run at start. It's optional and states ```true``` if it should.
- ```mode```: Describe where the application window should be. Examples: ```split-left``` or ```split-right``` 

The webapps contained for this project are the following:

- RvizWeb
- XPRA
- rosgraph
- rosboard
- webviz

RvizWeb and XPRA are also started by default to simplify the process of running a jupyter notebook. The following is an example of the existent json file `webapps.json` which configures the described environment:

```
[
  {
    "name": "rvizweb",
    "title": "Rvizweb",
    "icon": "proxy/8001/rvizweb/webapps/r.svg",
    "url": "proxy/8001/rvizweb/webapps/rvizweb/build/www/index.html",
    "start": true,
    "mode": "split-left"
  },
  {
    "name": "XPRA",
    "title": "Xpra Desktop",
    "icon": "proxy/8001/rvizweb/webapps/xpra-logo.svg",
    "url": "xprahtml5/index.html",
    "start": true,
    "mode": "split-right"
  },
  
  {
    "name": "rosgraph",
    "title": "Ros Graph",
    "icon": "proxy/8001/rvizweb/webapps/o.svg",
    "url": "proxy/8001/rvizweb/webapps/ros-node-graph/build/index.html"
  },
  {
    "name": "rosboard",
    "title": "ROSBoard",
    "icon": "proxy/8001/rvizweb/webapps/s.svg",
    "url": "proxy/18888/index.html"
  },
  {
    "name": "webviz",
    "title": "Webviz",
    "icon": "proxy/8001/rvizweb/webapps/webviz/icon.svg",
    "url": "proxy/8001/rvizweb/webapps/webviz/index.html"
  }
]
```


<a name="rvizweb">
<li><b><h3>  Custom RvizWeb configuration </h3></b>

Adding an RvizWeb is relative similar to adding a local rviz configuration. Although the config file structure differ (RvizWeb uses json and not the rviz format) which is why the <b>config file needs to be created using RvizWeb</b>. When first setting up a config, this can then be copied by clicking on <b> Load Config</b>:


<p align="center">
  <img src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/rvizweb_load_options.png">
</p>

After that a window similar to the following image should open:

<p align="center">
  <img height=500 src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/rvizweb_config.png">
</p>

This can then be copied into a local file. <b> Delete </b> lines containing the parameters <b>url, colladaServer, and videoServer</b> as they change with every startup and will be assigned automatically. 

<b> Note: When using a custom RvizWeb configuration, ensure that the name of the file match with the config file specified in the [entrypoint](#entry).</b>

</li>

<a name="xpra">
<li><b><h3>XPRA </h3></b>
XPRA is a remote display server which can be used to demonstrate the execution when using the Bulletworld. In the shown <b>webapps</b> file, it is a starting window on the left by default


</li>


<br>


<!--------------------Tutorial------------------------>

<a name="tutorial">
<summary style="text-align: center;"><b><h1>
                       Tutorials
</h1></b></summary>


<a name="pickup">
<summary style="text-align: center;"><b><h2> 
                      Pick up Example
</h2></b></summary>

A short example is shown here with Initializing a world with a robot, spawning and then picking up an object. This is written in an jupyter notebook in the file <a href="https://github.com/K3cks/pycram/blob/binder-example/examples/pick-test.ipynb">pick-test.ipynb</a>. There is also one cell which can create an <b> TFBroadcaster, a JointPublisher and a VisualizationMarkerPublisher</b>

<li><b><h3> 
                      Initialize World, Robot and object 
</h3></b></li>
When executing the initialization, the Bulletworld should open on XPRA and every action can be seen there as well. 
This is seperated into 3 different cells for clarification on what each action does. Alternatively the Bulletworld can also be initialized with a Parameter <b>'DIRECT'</b> if working with XPRA is not desired. This will not open any windows and execute everything without direct visualization. Although it is possible to use the <b>TFBroadcaster, JointPublisher or VisualizationMarker to visualize </b> the current execution in RvizWeb  

The code for this is seen below: 
```
from rospy import get_param
from pycram.bullet_world import BulletWorld, Object
from pycram.pose import Pose

from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.language import macros, par
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.plan_failures import IKError
import math

try:
    robot = get_param('/nbparam_robot')
    environment = get_param('/nbparam_environment')
except Exception as e:
    robot = 'pr2'
    environment = 'kitchen'
    
print(f"Robot: {robot}")
print(f"Environment: {environment}")

world = BulletWorld()

# Initialize Robot
robot = Object("pr2", "robot", robot + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
 
# Create Object
milk = Object("milk", "milk", "milk.stl", pose=Pose([2, 0, 1]))
milk_BO = BelieveObject(names=["milk"])
```

<li><b><h3> 
                      Creating Several Publisher
</h3></b></li>

In case any Publisher is wanted, it is neccessary to prepare a display for that depending on the type of Visualization is desired to use:

<summary><b><h4>
                      TFBroadcaster
</h4></b></summary>
 
To set up the TFBroadcaster simply add the display called <b> TF </b>. If you want to visualize all tf-frames, then you do not need to add a prefix. The following picture shows the location of the TF display: 

<p align="center">
  <img src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/TFBroadcaster.png">
</p>

Then the following line of code adds the TFBroadcaster. As soon as this is called, it publishes <b> all </b> available tf-frames:
<br>
`broadcaster = TFBroadcaster()`



<summary><b><h4>
                      Joint State Publisher
</h4></b></summary>
To set up the Joint State Publisher for a robot add the display called <b> Robot Model </b>. The robot description should be called <b> robot_description</b>. The tf-prefix might variate, the starting prefix so far is <b> simulated/pr2_2</b>. The following picture shows the setup of this display: 

<p align="center">
  <img src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/JointStatePublisher.png">
</p>

Then the following line of code adds the JointStatePublisher. As soon as this is called, it publishes the joint states under the given name (here: <b> joint_states</b>)
<br>
`joint_publisher = JointStatePublisher("joint_states", 0.1)`



<summary><b><h4> 
                      VisualizationMarker
</h4></b></summary>
To set up the Visualization add the disply called <b> Marker Array </b>. It is required to set a topic name which is currently <b> /viz_marker</b>. The following picture shows the setup of this display: 

<p align="center">
  <img src="https://raw.githubusercontent.com/K3cks/pycram/binder-example/binder/Readme/VisualizationMarker.png">
</p>

Then the following line of code adds the VisualizationMarker. Here all markers should be published under the given name (here: <b> /viz_marker</b>)
<br>
`v = VizMarkerPublisher(topic_name='viz_marker')`

<b> NOTE: This is currently bugged in RvizWeb and needs further investigation. Visualizationmarker does work locally, but only shows a bugged model in RvizWeb.</b>

<li><b><h3> 
                      Navigate to Object
</h3></b></li>

The navigation to objects does not require any setup specifically for binder so there is no difference in execution:

```
with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.33]).resolve().perform()

    pickup_pose_knife = CostmapLocation(target=milk_BO.resolve(), reachable_for=robot_desig).resolve()
    pickup_arm = pickup_pose_knife.reachable_arms[0]

    NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()

```


<li><b><h3> 
                      Pick up object
</h3></b></li>
Next pick up the object with the left arm. This also does not differ from normal pycram usage:

```
with simulated_robot:
    PickUpAction(object_designator_description=milk_BO,
                 arms=["left"],
                 grasps=["left", "right"]).resolve().perform()
```












