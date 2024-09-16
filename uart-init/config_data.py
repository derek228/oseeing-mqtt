import serial
import time

RS485_SET_DEVICE_ID	=	0x80
RS485_SET_DEVICE_MODE=	0x81
RS485_SET_FRAME     =   0x70
RS485_SET_SQUARE1	=	0x71
RS485_SET_SQUARE2	=	0x72
RS485_SET_SQUARE3	=	0x73
RS485_SET_SQUARE4	=	0x74
RS485_SET_SQUARE5	=	0x75
RS485_SET_SQUARE6	=	0x76	
RS485_SET_SQUARE7	=	0x77
RS485_SET_SQUARE8	=	0x78
RS485_SET_SQUARE9	=	0x79
RS485_GET_ALARM_STATUS = 0x82 # return id , alarm, max length 20 bytes
RS485_GET_FRAME_STATUS=	0x84 # return max, min
RS485_GET_SQUARE_STAUTS=	0x83 # return max, min
BAUDRATE=9600 #115200
RESET_DEVICE_ID={0xAA,0xFF,0x5A,0xA5,0x03,0x24}
class SerialConfig:

    def __init__(self):
        self.com_port=None
        self.connected=0
        self.mode='frame'
        self.id=0xAA
        self.frame_alarm=0
        self.frame_alarm_temperature=0
        self.square1_alarm=0
        self.square1_alarm_temperature=0
        self.square2_alarm=0
        self.square2_alarm_temperature=0
        self.square3_alarm=0
        self.square3_alarm_temperature=0
        self.square4_alarm=0
        self.square4_alarm_temperature=0
        self.square5_alarm=0
        self.square5_alarm_temperature=0
        self.square6_alarm=0
        self.square6_alarm_temperature=0
        self.square7_alarm=0
        self.square7_alarm_temperature=0
        self.square8_alarm=0
        self.square8_alarm_temperature=0
        self.square9_alarm=0
        self.square9_alarm_temperature=0
        self.transfer_port='RS485'

    def set_id(self,id):
        self.id=id

    def set_serial(self,port):
        self.com_port=port

    def set_frame_alarm_temperature(self,val):
        self.frame_alarm_temperature=val
    def set_frame_alarm(self,val):
        self.frame_alarm=val
    def set_square_alarm(self,val,idx):
        if (idx==1) :
            self.square1_alarm=val
        elif (idx==2) :
            self.square2_alarm=val
        elif (idx==3) :
            self.square3_alarm=val
        elif (idx==4) :
            self.square4_alarm=val
        elif (idx==5) :
            self.square5_alarm=val
        elif (idx==6) :
            self.square6_alarm=val
        elif (idx==7) :
            self.square7_alarm=val
        elif (idx==8) :
            self.square8_alarm=val
        elif (idx==9) :
            self.square9_alarm=val

    def set_square_alarm_temperature(self,val,idx):
        if (idx==1) :
            self.square1_alarm_temperature=val
        elif (idx==2) :
            self.square2_alarm_temperature=val
        elif (idx==3) :
            self.square3_alarm_temperature=val
        elif (idx==4) :
            self.square4_alarm_temperature=val
        elif (idx==5) :
            self.square5_alarm_temperature=val
        elif (idx==6) :
            self.square6_alarm_temperature=val
        elif (idx==7) :
            self.square7_alarm_temperature=val
        elif (idx==8) :
            self.square8_alarm_temperature=val
        elif (idx==9) :
            self.square9_alarm_temperature=val

    def print_oseeing_config(self) :
        print(self)

    def reset_device_id(self) :
        data_packet = bytearray([0xAA,0xFF,0x5A,0xA5,0x03,0x24])
        return self.send_and_receive_data(data_packet) 
    
    def set_square(self, index, data) :
        command_id = RS485_SET_FRAME+index
        data_packet = bytearray([int(self.id,16), command_id, 1, data])
        return self.send_and_receive_data(data_packet)   
    #set frame
    def set_device_mode(self) :
        if self.frame_alarm == '1':
            #data_packet = bytearray([int(self.id,16), RS485_SET_DEVICE_MODE, 1, int(self.frame_alarm_temperature)])
            data_packet = bytearray([int(self.id,16), RS485_SET_DEVICE_MODE, 1, 1])
        else :
            data_packet = bytearray([int(self.id,16), RS485_SET_DEVICE_MODE, 1, 0])
        return self.send_and_receive_data(data_packet)   

    def set_device_id(self, id) :
        #data_packet = bytearray([self.id, RS485_SET_DEVICE_ID, 1, id])
        data_packet = bytearray([int(self.id,16), RS485_SET_DEVICE_ID, 1, id])
        return self.send_and_receive_data(data_packet)   

    def send_and_receive_data(self, data_packet):
        retry=30
        print(f"{self.com_port} Baudrate={BAUDRATE}")
        with serial.Serial(self.com_port, BAUDRATE, timeout=1) as ser:
            ser.flushInput() 
            ser.flushOutput()
            
            ser.write(data_packet)
            print("Data sent:", data_packet)
            
            while retry:
                received_data = ser.read(len(data_packet))
            
                if received_data == data_packet:
                    print("Received data matches sent data:", received_data)
                    return True
                    break;
                else:
                    print("Received data does not match sent data or no data received.")
                    print(received_data)
                    retry -= 1
                if retry == 0 :
                    print(f"Set command({data_packet}) fail....")
                    return False
            time.sleep(1)
