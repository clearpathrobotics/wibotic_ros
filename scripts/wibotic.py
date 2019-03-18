#!/usr/bin/python
import rospy
from wibotic_ros.msg import Wibotic as wibotic_msg
import threading
import time
from ws4py.client.threadedclient import WebSocketClient
from ws4py.messaging import BinaryMessage
import wibotic_msg_funcs
import packet_tools
import binascii

paramToRead = ["EthIPAddr", "DevMACOUI", "DevMACSpecific"] #note this is setup only for TX, won't automatically get pushed to rostopic
macaddress = []
ip_address = ''
ros_topic = ''
rostopicFrequency = 0

class ros_message:
    def __init__(self):
        self.msg = wibotic_msg()
        self.lock = threading.Lock()
        self.last_time_obtained = rospy.Time.now()
        self.time_pushed = rospy.Time.now()


    def push_to_msg(self, device, param, data):
        with self.lock:
            self.time_pushed = rospy.Time.now()
            self.msg = wibotic_msg_funcs.push_to_wibotic_msg(device, param, data, self.msg) # switch-case of setter functions

    def get_current_msg(self):
        with self.lock:
            if (self.time_pushed >= self.last_time_obtained): #if no new messages return empty message (these dont publish)
                self.last_time_obtained = rospy.Time.now()
                return self.msg
            return wibotic_msg()

    def set_to_zero(self):
        empty_message = wibotic_msg()
        self.msg = empty_message

class OpenClient(WebSocketClient):
    def opened(self):
        rospy.loginfo(">> Websocket Opened")
        for req in paramToRead:
            p = BinaryMessage(packet_tools.build_read_request("TX",req))
            self.send(p)
        #Note: like above a build_write_request can also be created

    def closed(self, code, reason=None):
        msg.set_to_zero()
        rospy.loginfo(">> Websocket Closed")
        rospy.loginfo ("Code: " + str(code) + "Reason: " + str(reason))

    def received_message(self, message):
        # rospy.loginfo(">> received_message @ " + str(rospy.Time.now().to_nsec()) + "\n")
        data = []
        for x in range(0, len(message.data)):
            data.append(binascii.hexlify(message.data[x]))
        for returned in packet_tools.process_data(data):
            device, param, value = returned.get_data()
            # rospy.loginfo(device + " " + param + " " + str(value)) #uncommenting prints all processed received messages

            if (value == "param_update" or param == "unrecognized"): #this returns confirmation of param update (no change to msg)
                continue
            if (param != "DevMACOUI" and param != "DevMACSpecific"):
                msg.push_to_msg(device, param, value)
            else:
                #Note: macaddress comes in 2 messages
                macaddress.append(value[2])
                macaddress.append(value[1])
                macaddress.append(value[0])
                if (len(macaddress) == 6):
                    # rospy.loginfo("MacAddress Built")
                    msg.push_to_msg(device, "MacAddress", macaddress)


def check_params():
    ip_address = rospy.get_param('~ip_address', 'ws://192.168.2.20/ws')
    rostopicFrequency = rospy.get_param('~topic_frequency', 1) #default to 1
    ros_topic = rospy.get_param('~ros_topic', 'wibotic_websocket')
    if (rostopicFrequency > 10):
        rospy.logfatal('~topic_frequency can\'t be set larger than 10')
        return
    if (str(ip_address) != 'ws://192.168.2.20/ws'):
        rospy.loginfo("Make sure websocket open on ip: " + str(ip_address))
    return (str(ip_address), rostopicFrequency, str(ros_topic))

def websocket(thread_name, ip):
    t = threading.currentThread()
    try:
        ws = OpenClient(ip, protocols=['wibotic'])
        ws.connect()
        while getattr(t, "still_run", True) and not ws.terminated:
            time.sleep(.5)
        ws.close()
    except KeyboardInterrupt:
        ws.close()

def start_websocket(ip):
    t = threading.Thread(target=websocket, args=("websocket_thread",ip)) #running ROS in another thread
    t.still_run = True
    t.start()
    return (t)

if __name__ == '__main__':
    rospy.init_node('Wibotic', anonymous=True)
    ip_address, rostopicFrequency, ros_topic = check_params()
    msg = ros_message()
    t = start_websocket(ip_address)
    pub = rospy.Publisher(ros_topic, wibotic_msg, queue_size=10)
    rate = rospy.Rate(rostopicFrequency) # controls how often to publish
    disconnect_counter = 0
    while not rospy.is_shutdown():
        message = msg.get_current_msg()
        if (message.TX.PacketCount != 0):
            disconnect_counter = 0
            message.header.stamp = rospy.Time.now()
            pub.publish(message) #Message is updated as data is received
        else:
            disconnect_counter += 1
        if disconnect_counter > 10:
            rospy.logwarn('Restarting WebSocketClient')
            disconnect_counter = 0
            while t.isAlive():
                t.still_run = False
                t.join(timeout=5)
            t = start_websocket(ip_address)
        rate.sleep()
    t.still_run = False
    t.join(timeout=5)
