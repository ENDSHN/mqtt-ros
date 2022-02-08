# MQTT-ROS
## Table of Contents
- **[Installation of the Environment](#installation-of-the-environment)**
- **[Docker-Compose Files](#docker-compose-files)**


# Installation of the Environment
An installation of Docker and Docker-Compose is required in order to build and start the environment.

For a basic test the `docker-compose.yml` image can be used. In order to start building the image, you can use:
```console
docker-compose build
```

Before testing the image it is required to change the parameter `MQTT_BROKER_ADDRESS` for each of the containers it is used in. This parameter can be found in each of the Docker-Compose files. It has to match the IP address of the MQTT Broker. Optionally the name of the robot can be changed as well by modifying the parameter `ROBOT_NAME`. Make sure that the name will be the same for each component.

Additionally a MQTT Broker must be started beforehand. This repository includes an example broker, which uses EMQX. To start it, run:
```console
docker-compose -f docker-compose-emqx-test.yml up
```

After it has been successfully built you can test it by using:
```console
docker-compose up
```

This will start 7 different Docker containers. Those are the MQTT-ROS environment for one robot, called robot0 as well as an additional Turtlebot 3 Simulator.

<br/>
<div align="right">
    <b><a href="#mqtt-ros">↥ back to top</a></b>
</div>
<br/>

# Docker-Compose Files
This repository includes a few different docker-compose files. Each of them have a different purpose and may also require specific environments to work, such as the case for the Turtlebot 3 images.

`docker-compose.yml` is the file that starts a robot instance with a Turtlebot 3 Simulator while also starting a server instance.

`docker-compose-emqx-test.yaml` is the file for the test MQTT Broker which starts an EMQX Broker.

`docker-compose-server.yml` starts only a server instance without any robot.

`docker-compose-tb3.yml` is used for the Turtlebot 3 itself and is required to run on a Turtlebot 3 as it is not a simulator.

<br/>
<div align="right">
    <b><a href="#mqtt-ros">↥ back to top</a></b>
</div>
<br/>