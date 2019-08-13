import serial.tools.list_ports
import multiprocessing, threading
import serial
import time
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
    def write(self, text):
        text += "\r\n"
        self.arduino_serial.write(text.encode())

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
                print(serial_data)
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
    # #<servo number>P<position number>
    def move_servo(self, servo, pos):
        if self.degree_type == "std":
            pos = self.map_degree_interval(pos)
        elif self.degree_type != "pomoco":
            raise Exception(f"degree type {self.degree_type} doesn`t allowed")
        self.robot_serial.write(f"#{servo}P{pos}")

    # C 
    def center_servos(self):
        self.robot_serial.write(f"C")

    # #<servo number>L
    def kill_servo(self, servo):
        self.robot_serial.write(f"#{servo}L")

    # K
    def kill_all_servos(self):
        self.robot_serial.write("K")

    # V
    def get_version(self):
        self.robot_serial.write("V")
        return self.robot_serial.read()

    # S<pin designator><pin option>
    def set_pin(self, pin, option):
        self.robot_serial.write(f"{pin}{option}")

    # #<Servo Number>0<Offset Value>
    def set_offset(self, servo, offset):
        self.robot_serial.write(f"#{servo}0{offset}")

    # $<4 byte mask><N byte send>
    def binary_mode_servo_move(self):
        print("Now this function doesn`t supported")
        pass

    # U<number of measurements>
    def ultrasonic(self, mes_num):
        self.robot_serial.write(f"U{mes_num}")

    def map_degree_interval(self, input):
        out_start = 500
        out_end = 2500
        inp_end = 180
        inp_start = 0
        input += 90
        return int(out_start + ((out_end - out_start) / (inp_end - inp_start)) * (input - inp_start))

    def close_connection(self):
        self.robot_serial.close()

    def __init__(self, *args, **kwargs):
        robot_serial_config = kwargs.get("serial", {})
        robot_serial_config["device_id"] = "VID:PID=27C2:0002"
        self.degree_type = kwargs.get("degree_type", "std")
        self.robot_serial = SerialConnection(**robot_serial_config)


def wave():
    s = ServotorAPI(degree_type="std")
    s.center_servos()
    for servo in (5, 9, 13, 18, 22, 26):
        s.move_servo(servo, -75)
    time.sleep(2)
    
    for i in range(10, 50, 3):
        for servo in (5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 18, 20, 21, 22, 24, 25, 26, 31):
            s.move_servo(servo, i)

        for servo in (5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 18, 20, 21, 22, 24, 25, 26, 31):
            s.move_servo(servo, -i)

    s.kill_all_servos()
    s.close_connection()


if __name__ == "__main__":
    s = ServotorAPI(degree_type="std")
    s.center_servos()
    s.move_servo(5, 90)
    #for i in range(25):
    #    s.move_servo(6, 45)
    s.move_servo(5, 45)
    #s.move_servo(6, -45)
