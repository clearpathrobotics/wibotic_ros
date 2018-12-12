<h1>Wibotic ROS Package</h1>

<h2>Description:</h2>
This package uses ws4py in a python script to retrieve data from the wibotic websocket. 
The data recieved is in binary format and has to be manipulated to make usable. The data 
is then organized and pushed to the '/wibotic_websocket' rostopic. Functions have also been 
included to send read and write requests for specific parameters. For more details on 
parameters look at the WiBotic_Network_API.pdf doc.

<h2>How to use:</h2>
1. Add this to a ros project.
2. run **rosdep update**
3. run **rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y** Note: change kinetic to your distro
4. source devel/setup.bash
5. run **rosrun wibotic_ros wibotic.py** to start publishing socket data to rostopic.

<h2>Parameters:</h2>
- topic_frequency: (default=1) If included this param allows user to control the frequency of messages published to rostopic
- ip_address: (default='ws://192.168.2.20/ws') If included this param allows user to pass in websocket ip in scenario that it has been changed from the default
- ros_topic: (default=wibotic_websocket) If included the topic on which messages are published will be changed

ex. **rosrun wibotic_ros wibotic.py _topic_frequency:=10 _ip_address:=ws://192.168.2.20/ws _ros_topic:=wibotic_new_topic**

<h2>Checkout the test folder for:</h2>
- Wibotic.html: an html tool for basic websocket communication
- WiBotic_Network_API.pdf: doc for Wibotic websocket api

