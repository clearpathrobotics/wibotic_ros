#!/usr/bin/python
import struct
import binascii

def DeviceID(device):
    switcher = {
        '01': "TX",
        '02': "RX"
    }
    return switcher.get(device)

class _ResponseType(int):
    #WARNING: switched first 2 values(not sure why websocket behaving weirdly)
    PARAM_UPDATE = 81
    PARAM_RESPONSE = 80
    ADC_UPDATE = 82

def AdcID(id):
    # get adc param name from id
    switcher = {
        0: "PacketCount",
        1: "ChargeState",
        2: "Flags",
        3: "PowerLevel",
        4: "VMon3v3",
        5: "VMon5v",
        6: "IMon5v",
        7: "VMon12v",
        8: "IMon12v",
        9: "VMonGateDriver",
        10: "IMonGateDriver",
        11: "VMonPA",
        12: "IMonPa",
        13: "TMonPa",
        14: "VMonBatt",
        15: "VMonBattProg",
        16: "VRect",
        17: "TBoard",
        18: "ICharger",
        19: "IBattery",
        20: "TargetIBatt",
        21: "IMaster",
        22: "ISlave1",
        23: "ISlave2",
    }
    return switcher.get(id)

def AdcTypes(param):
    # get adc param type from name
    switcher = {
        "PacketCount":"uint32_t",
        "ChargeState":"uint8_t",
        "Flags":"uint16_t",
        "PowerLevel":"uint16_t",
        "VMon3v3":"float",
        "VMon5v":"float",
        "IMon5v":"float",
        "VMon12v":"float",
        "IMon12v":"float",
        "VMonGateDriver":"float",
        "IMonGateDriver":"float",
        "VMonPA":"float",
        "IMonPa":"float",
        "TMonPa":"float",
        "VMonBatt":"float",
        "VMonBattProg":"float",
        "VRect":"float",
        "TBoard":"float",
        "ICharger":"float",
        "IBattery":"float",
        "TargetIBatt":"float",
        "IMaster":"float",
        "ISlave1":"float",
        "ISlave2":"float"
    }
    return switcher.get(param)

params = {
        "Address":"03",
        "RadioChannel":"04",
        "DigitalBoardVersion":"1a",
        "BatteryCurrentMax":"22",
        "ChargerCurrentLimit":"23",
        "MobileRxVoltageLimit":"24",
        "RxBatteryVoltageMin":"25",
        "BuildHash":"26",
        "TargetFirmwareId":"27",
        "RxBatteryVoltage":"2a",
        "RxBatteryCurrent":"2b",
        "RxTemperature":"2c",
        "EthIPAddr":"2d",
        "EthNetMask":"2e",
        "EthGateway":"2f",
        "EthDNS":"30",
        "EthUseDHCP":"31",
        "EthUseLLA":"32",
        "DevMACOUI":"33",
        "DevMACSpecific":"34",
        "EthMTU":"36",
        "EthICMPReply":"37",
        "EthTCPTTL":"38",
        "EthUDPTTL":"39",
        "EthUseDNS":"3a",
        "EthTCPKeepAlive":"3b",
        "ChargeEnable":"3c",
        "I2cAddress":"3d",
        "RxBatteryNumCells":"3e",
        "RxBatterymVPerCell":"3f",
        "LogEnable":"43",
        "RxBatteryChemistry":"44",
        "IgnoreBatteryCondition":"46",
        "PowerBoardVersion":"47",
        "AccessLevel":"4e",
    }

def getParamID(param):
    # get param id from name
    return params.get(param)

def getParamFromID(id):
    # get param name from id
    for param in params:
        if (params.get(param) == id):
            return param
    return 0

class ADCUpdate:
    """ Contains new ADC values that are sent periodically """
    def __init__(self, device, param, data):
        self.device = device
        self.param = param
        self.data = data
        
    def get_data(self):
        #Gets hex and changes to type based on param
        adc_type = AdcTypes(str(self.param))
        hex = self.data[3] + self.data[2] + self.data[1] + self.data[0] #little endian
        if (adc_type == "float"):
            value = struct.unpack('!f', hex.decode('hex'))[0]
        else:
            value = int(hex, 16)
        
        return (self.device,str(self.param),str(value)) #returns tuple

class ParamResponse:
    """ Contains new param data returned """
    def __init__(self, device, param, data):
        self.device = device
        self.param = param
        self.data = data
        
    def get_data(self):
        # adc_type = AdcTypes(str(self.param))
        # hex = self.data[3] + self.data[2] + self.data[1] + self.data[0]
        # if (adc_type == "float"):
        #     value = struct.unpack('!f', hex.decode('hex'))[0]
        # else:
        #     value = int(hex, 16)
        return (self.device,str(self.param),self.data) #returns tuple, note data is array of returned hex vals

class ParamUpdate:
    """ Contains new param data returned """
    def __init__(self, device, param):
        self.device = device
        self.param = param
        
    def get_data(self):
        return (self.device,str(self.param),"param_update") #returns tuple, note data is array of returned hex vals

def process_data(data):
    """ Takes binary data and processes it into an object that can be 
    easily parsed """
    def param_update(data):
        #WARNING: _ResponseType values were switched!
        device_id = DeviceID(data.pop(0)) #popped id
        response_id = getParamFromID(data[0]) #get data id
        return [ParamUpdate(device_id, response_id)] #returns array with only single object
        
    def param_response(data):
        #WARNING: _ResponseType values were switched!
        device_id = DeviceID(data.pop(0)) #popped id
        response_id = getParamFromID(data[0]) #get data id
        param_data = []
        for i in range(4, 8): #push data into an array
            param_data.append(data[i])
        return [ParamResponse(device_id, response_id, param_data)] #returns array with only single object
        
    def adc_update(data):
        device_id = DeviceID(data.pop(0)) #popped id, only real data left
        number_adc_data = len(data)//6
        all_adc_data = []
        for x in range(0, number_adc_data):
            data_location = x*6
            adc_id = AdcID(int(data[data_location], 16)) #get data id
            adc_data = [] #empty data array
            for i in range(2, 6): #Push actual data into array
                adc_data.append(data[data_location+i])
            all_adc_data.append(ADCUpdate(device_id, adc_id, adc_data))
        return all_adc_data
    
    #this calls functions depending on response_type
    event = {
        _ResponseType.PARAM_UPDATE: param_update,
        _ResponseType.PARAM_RESPONSE: param_response,
        _ResponseType.ADC_UPDATE: adc_update
    }
    response_type = _ResponseType(data.pop(0))
    return event[response_type](data)
    
def build_read_request(device, parameter):
    if device == "RX" or device == "Charger" or device == 2:
        device = '02';
    else:
        device = '01';
    parameter_id = getParamID(parameter)
    byte_string = '01' + device + '000000' + parameter_id

    print("built hex read request: " + str(byte_string))

    #get message ready to be made into BinaryMessage
    ba = (binascii.unhexlify(byte_string))
    print("message has been encoded")
    return ba

def build_write_request(device, parameter, data):
    if device == ("RX" or "Charger" or 2):
        device = '02';
    else:
        device = '01';
    parameter_id = getParamID(parameter)

    # example hex data string: data = '01234567'
    if (len(data) != 8): #checks if data is correct
        return -1

    lil_end_data = data[6] + data[7] + data[4] + data[5] + data[2] + data[3] + data[0] + data[1] #little endian of hex data string
    byte_string = '03' + device + '000000' + parameter_id + lil_end_data

    print("built hex write request: " + str(byte_string))

    #get message ready to be made into BinaryMessage
    ba = (binascii.unhexlify(byte_string))
    print("message has been encoded")
    return ba