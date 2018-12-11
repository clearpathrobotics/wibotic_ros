#!/usr/bin/python

def turn_to_uint(string, bit_range):
    i = int(string)
    return i % 2**bit_range 

########################## TX ##########################

def tx_EthIPAddr(msg, data):
    msg.TX.EthIPAddr = str(int(data[3],16)) + "." + str(int(data[2],16)) + "." + str(int(data[1],16)) + "." + str(int(data[0],16))
    return msg

def tx_MacAddress(msg, data):
    msg.TX.MacAddress = str(data[0]) + ":" + str(data[1]) + ":" + str(data[2]) + ":" + str(data[3]) + ":" + str(data[4]) + ":" + str(data[5])
    return msg

def tx_PacketCount(msg, data):
    msg.TX.PacketCount = turn_to_uint(data, 32) #turns to uint32
    return msg

def tx_ChargeState(msg, data):
    msg.TX.ChargeState = turn_to_uint(data, 8) #turns to uint8
    return msg

def tx_Flags(msg, data):
    msg.TX.Flags = turn_to_uint(data, 16) #turns to uint16
    return msg

def tx_VMon3v3(msg, data):
    msg.TX.VMon3v3 = float(data)
    return msg

def tx_VMon5v(msg, data):
    msg.TX.VMon5v = float(data)
    return msg

def tx_IMon5v(msg, data):
    msg.TX.IMon5v = float(data)
    return msg

def tx_VMon12v(msg, data):
    msg.TX.VMon12v = float(data)
    return msg

def tx_IMon12v(msg, data):
    msg.TX.IMon12v = float(data)
    return msg

def tx_VMonGateDriver(msg, data):
    msg.TX.VMonGateDriver = float(data)
    return msg

def tx_IMonGateDriver(msg, data):
    msg.TX.IMonGateDriver = float(data)
    return msg

def tx_VMonPA(msg, data):
    msg.TX.VMonPA = float(data)
    return msg

def tx_IMonPa(msg, data):
    msg.TX.IMonPa = float(data)
    return msg

def tx_TMonPa(msg, data):
    msg.TX.TMonPa = float(data)
    return msg

def tx_VMonBatt(msg, data):
    msg.TX.VMonBatt = float(data)
    return msg

########################## RX ##########################

def rx_EthIPAddr(msg, data):
    msg.RX.EthIPAddr = str(int(data[3],16)) + "." + str(int(data[2],16)) + "." + str(int(data[1],16)) + "." + str(int(data[0],16))
    return msg

def rx_MacAddress(msg, data):
    msg.RX.MacAddress.MacAddress = str(data[0]) + ":" + str(data[1]) + ":" + str(data[2]) + ":" + str(data[3]) + ":" + str(data[4]) + ":" + str(data[5])
    return msg

def rx_PacketCount(msg, data):
    msg.RX.PacketCount = turn_to_uint(data, 32) #turns to uint32
    return msg

def rx_ChargeState(msg, data):
    msg.RX.ChargeState = turn_to_uint(data, 8) #turns to uint8
    return msg

def rx_Flags(msg, data):
    msg.RX.Flags = turn_to_uint(data, 16) #turns to uint16
    return msg

def rx_PowerLevel(msg, data):
    msg.RX.PowerLevel = turn_to_uint(data, 16) #turns to uint16
    return msg

def rx_VMonBattProg(msg, data):
    msg.RX.VMonBattProg = float(data)
    return msg

def rx_VRect(msg, data):
    msg.RX.VRect = float(data)
    return msg

def rx_TBoard(msg, data):
    msg.RX.TBoard = float(data)
    return msg

def rx_ICharger(msg, data):
    msg.RX.ICharger = float(data)
    return msg

def rx_IBattery(msg, data):
    msg.RX.IBattery = float(data)
    return msg

def rx_TargetIBatt(msg, data):
    msg.RX.TargetIBatt = float(data)
    return msg

def rx_IMaster(msg, data):
    msg.RX.IMaster = float(data)
    return msg

###################### Actual switch for funcs #####################

def push_to_wibotic_msg(device, param, data, msg):
    # msg.header.stamp = ros_time
    if (device == "TX"):
        # msg.TX.header.stamp = ros_time
        msg.TX.device = device
        switcher = {
            "EthIPAddr": tx_EthIPAddr,
            "MacAddress": tx_MacAddress,
            "PacketCount":tx_PacketCount,
            "ChargeState":tx_ChargeState,
            "Flags":tx_Flags,
            "VMon3v3":tx_VMon3v3,
            "VMon5v":tx_VMon5v,
            "IMon5v":tx_IMon5v,
            "VMon12v":tx_VMon12v,
            "IMon12v":tx_IMon12v,
            "VMonGateDriver":tx_VMonGateDriver,
            "IMonGateDriver":tx_IMonGateDriver,
            "VMonPA":tx_VMonPA,
            "IMonPa":tx_IMonPa,
            "TMonPa":tx_TMonPa,
            "VMonBatt":tx_VMonBatt,
        }
    else:
        # msg.RX.header.stamp = ros_time
        msg.RX.device = device
        switcher = {
            "EthIPAddr": rx_EthIPAddr,
            "MacAddress": rx_MacAddress,
            "PacketCount":rx_PacketCount,
            "ChargeState":rx_ChargeState,
            "Flags":rx_Flags,
            "PowerLevel":rx_PowerLevel,
            "VMonBattProg":rx_VMonBattProg,
            "VRect":rx_VRect,
            "TBoard":rx_TBoard,
            "ICharger":rx_ICharger,
            "IBattery":rx_IBattery,
            "TargetIBatt":rx_TargetIBatt,
            "IMaster":rx_IMaster,
        }
    func = switcher.get(param, "No function") #if param doesnt exist this should just return original msg
    if (func == "No function"):
        return msg
    msg = func(msg, data)
    return msg