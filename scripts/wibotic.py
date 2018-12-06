#!/usr/bin/python
import rospy
from wibotic_ros.msg import Wibotic as wibotic_msg
import thread
from ws4py.client.threadedclient import WebSocketClient
from ws4py.messaging import BinaryMessage
import wibotic_msg_funcs
import packet_tools
import binascii

macaddress = []

class ros_message:
    def __init__(self):
        self.msg = wibotic_msg()

    def push_to_msg(self, device, param, data):
        self.msg = wibotic_msg_funcs.push_to_wibotic_msg(device, param, data, self.msg) # switch-case of setter functions

    def get_current_msg(self):
        return self.msg

class OpenClient(WebSocketClient):
    def opened(self):
        print (">> opened")
        reqToSend = ["EthIPAddr", "DevMACOUI", "DevMACSpecific"] #note this is setup only for TX
        for req in reqToSend:
            p = BinaryMessage(packet_tools.build_read_request("TX",req))
            self.send(p)
        #Note: like above a build_write_request can also be created

    def closed(self, code, reason=None):
        print (">> closed")
        print "Closed down", code, reason

    def received_message(self, message):
        # print (">> received_message\n")
        data = []
        for x in range(0, len(message.data)):
            data.append(binascii.hexlify(message.data[x]))
        for returned in packet_tools.process_data(data):
            device, param, value = returned.get_data()
            # print (device + " " + param + " " + str(value)) #uncommenting prints all processed received messages

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
                    print("Mac Built")
                    msg.push_to_msg(device, "MacAddress", macaddress)

def ros_setup():
    pub = rospy.Publisher('wibotic_websocket', wibotic_msg, queue_size=10)
    rospy.init_node('Wibotic', anonymous=True)
    rate = rospy.Rate(10) # 10hz, controls how often to publish
    while not rospy.is_shutdown():
        message = msg.get_current_msg()
        message.header.stamp = rospy.Time.now()
        pub.publish(message) #Message is updated as data is received

def websocket(thread_name):
    try:
        ws = OpenClient('ws://192.168.2.20/ws', protocols=['wibotic'])
        ws.connect()
        ws.run_forever()
    except KeyboardInterrupt:
        ws.close()

if __name__ == '__main__':
    msg = ros_message()
    thread.start_new_thread(websocket, ("websocket_thread",)) #running ROS in another thread
    ros_setup();
