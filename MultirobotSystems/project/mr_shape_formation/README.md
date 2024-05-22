# mr_shape_formation

![portada_github_mrps](https://user-images.githubusercontent.com/105311357/167719727-949c0bac-26d7-4e3b-a66f-2690cdd3460b.jpeg)


## Requirements hola

This project runs with ROS, and Python 3.8 or above. Plus the `pygame` library is used so you need to have those installed. So if you don't have any of them please follow these steps:

- Install ROS Noetic: [ROS Official Webpage](http://wiki.ros.org/noetic/Installation/Ubuntu)

- Install Python 3.8 or above: [Python Official Webpage](https://docs.python-guide.org/starting/install3/linux/)

- Install (Once you have python installed)
`pip install -U pygame`


## Instalation: Project + Turtlebot2

Let's install the turtlebot2 so you can run this project:
```
mkdir $HOME/tb2_ws
cd ~/tb2_ws

wstool init src ../tb2.rosintall

rm -rf src/ar_track_alvar/ar_track_alvar

catkin_make_isolated
```

Now, it's the time to clone the repository
```
cd ~/tb2_ws/src
git clone https://github.com/coredumbs/multirobot-pattern-shaping.git

cd ..
catkin_make_isolated
source devel_isolated/setup.bash
```


In order to be able of running the programming you need to give execute permission to a couple of python src programs

```
chmod +x ~/tb2_ws/src/multirobot-pattern-shaping/src/interface.py 
chmod +x ~/tb2_ws/src/multirobot-pattern-shaping/src/lib_interface.py 
```

## Execution

Now everything is installed you have two options to run the project. There´s a centralized version and a decentralized version. The command for launching any of them it's quite the same for both:

- Centralized
`roslaunch multirobot-pattern-shaping centralized.launch`

- Decentralized
`roslaunch multirobot-pattern-shaping decentralized.launch`

## Errors that can appear
It's possible that in the you need to change the name of the icons of the interface. It may happen that names are changed during cloning. So please, go to the directory '/config/icons' and check that the filenames are: 
- coordinate_system.png
- info.png
- points_and_robots.png
- shapingpattern_icon.png
- title.png

