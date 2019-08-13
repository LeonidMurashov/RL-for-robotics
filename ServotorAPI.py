import serial.tools.list_ports
import multiprocessing, threading
import serial
import time
import json
import os, sys

#   Servos: 5, 6, 7, 9, 10, 11, 13, 14, 15
#           16, 17, 18, 20, 21, 22, 24, 25, 26, 31   
#   Pomoco: 
#       - move servo:
#           500 - 90 left
#           1500 - 0 center
#           2500 - 90 right
#

class SerialConnection():
    def write(self, msg):
        self.arduino_serial.write(msg)

    def read(self):
        print("Start listening serial process")

        while not self.reading_stopped.is_set():
            serial_data = ""
            try:
                serial_data = self.arduino_serial.readline()
            except:
                exc_type, err_msg = sys.exc_info()[:2]
                print(f"error reading port: {err_msg}")
                self.reading_stopped.set()
                break
            
            if len(serial_data) > 0:
                serial_data = serial_data.decode("utf-8")
                serial_data = str(serial_data).replace("\r\n", "")
                serial_data = serial_data.replace("\x000", "")
                self.reading_queue.put(serial_data)

        print("port listening process was stopped")
        self.arduino_serial.close()

    def receive(self):
        print("Start receiving data process")
        while not self.reading_stopped.is_set():
            if not self.reading_queue.empty():
                serial_data = self.reading_queue.get()
        print("recieving data process was stopped")

    def close(self):
        self.reading_stopped.set()
        self.arduino_serial.close()
        self.read_process.join()
        self.receive_process.join()
        print('serial connection stopped')
    
    def find_arduino_port(self):
        if (os.name == "nt"):
            ports = list(serial.tools.list_ports.comports())        
            for port in ports:
                if self.device_id in port[2]:
                    self.arduino_port = port[0]
                    return
            raise(Exception("arduino doesn`t connected"))
        else:
            arduino_port = os.popen("dmesg | egrep ttyACM | cut -f3 -d: | tail -n1").read().strip()
            if arduino_port == "":
                raise(Exception("arduino doesn`t connected"))
            else:
                self.arduino_port = "/dev/"+arduino_port

    def __init__(self, *args, **kwargs):
        self.device_id = kwargs.get("device_id", "arduino")
        self.find_arduino_port()

        self.arduino_serial = serial.Serial(
            port=self.arduino_port,
            baudrate=kwargs.get("baudrate", 9600),
            timeout=kwargs.get("timeout", 1),
        )

        self.reading_queue = multiprocessing.Queue()
        self.reading_stopped = threading.Event()
        self.read_process = threading.Thread(target=self.read)
        self.receive_process = threading.Thread(target=self.receive)

        self.read_process.start()
        self.receive_process.start()

class ServotorAPI():
    def kill_all_servos(self):
        self.rotate_servos([255]*18)

    def close_connection(self):
        self.robot_serial.close()

    def rotate_servos(self, angles):
        if len(angles) != 18:
            raise(Exception("not 18 servos"))

        for i in range(12, 18):
            angles[i] = (180 - angles[i]) if angles[i] <= 180 else 255
        self.robot_serial.write(bytes(angles))
        
    def reset(self):
        s.rotate_servos([-(i > 5) * 45 + 90 for i in range(18)])
        
    def __init__(self, *args, **kwargs):
        robot_serial_config = kwargs.get("serial", {})
        robot_serial_config["device_id"] = "VID:PID=2A03:0043"
        self.degree_type = kwargs.get("degree_type", "std")
        self.robot_serial = SerialConnection(**robot_serial_config)