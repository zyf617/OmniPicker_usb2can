import serial
import time

class OmniPicker_Interface():
    def __init__(self,
                 Can_ID=0x01,
                 serial_port='/dev/ttyUSB0',
                 baud_rate=115200,
                 timeout=1.0,
                 bytesize=8,
                 parity='N',
                 stopbits=1):
        '''
        Param:
        Can_ID(int): control ID of OmniPicker(NOT USED);
        serial_port(str): mount name of your usb2can;
        '''
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=timeout, bytesize=bytesize, parity=parity, stopbits=stopbits)
            
            # my USB-CAN adapter need to be designated as transparent transmission mode
            # if your adapter can use transparent transmission directly, just skip
            self.ser.write(b'AT+ET\r\n')    # sending AT protocol to use transparent transmission mode
            self.ser.flush()    # make sure the data has been sent
            print("Initialize Done!\n")
        except Exception as e:
            print(f"Failed to initialize: {e}.")
            exit()
        
    def __del__(self):
        self.ser.close()
        
    def control(self, Pos=0.0, For=100.0, Vel=100.0, Acc=100.0, Dec=100.0):
        """
        Send gripper control signal.
        
        Param:
        Pos(float): position control, range: [0, 100.0], 0 - fully closed, 100.0 - fully open;
        For(float): force control, range: [0, 100.0], 100.0 - maximum force;
        Vel(float): velocity control, range: [0, 100.0], 100.0 - maximum velocity;
        Acc(float): acceleration control, range: [0, 100.0], 100.0 - maximum acceleration;
        Dec(float): deceleration control, range: [0, 100.0], 100.0 - maximum deceleration;
        
        Return:
        Ret(int): fault code: 0 - no fault, 1 - over temperature, 2 - over speed, 3 - initialization failure, 4 - over limit;
        State(int): current state: 0 - target arrived, 1 - running, 2 - locked rotor, 3 - object fallen;
        Position(float): current position of OmniPicker, range: [0, 100.0];
        Velocity(float): current velocity of OmniPicker, range: [0, 100.0];
        Force(float): current force of OmniPicker, range: [0, 100.0];
        """
        data_list = [0,
                     int((max(0, min(100, Pos)) / 100) * 255),
                     int((max(0, min(100, For)) / 100) * 255),
                     int((max(0, min(100, Vel)) / 100) * 255),
                     int((max(0, min(100, Acc)) / 100) * 255),
                     int((max(0, min(100, Dec)) / 100) * 255),
                     0,
                     0
                     ]          # 0 - Reserved
        data = bytes(data_list)
        
        try:
            print(f"Sending data: {data.hex()}.")
            self.ser.write(data)
            self.ser.flush()    # make sure the data has been sent
            time.sleep(1.0)     # wait for the OmniPicker running
            
            # OmniPicker will return the feedback immediately once it receives the control signal
            # so send the control signal again to get the OmniPicker feedback after control.
            self.ser.write(data)
            self.ser.flush()
            time.sleep(0.1)
            
            feedback_all = self.ser.read_all()
            feedback = feedback_all[-8:]
            print(f"Get feedback: {feedback.hex()}.")
        except Exception as e:
            print(f"Failed to send data or receive feedback: {e}.")
            
        Ret = feedback[0]
        State = feedback[1]
        Position = feedback[2] / 255.0 * 100.0
        Velocity = feedback[3] / 255.0 * 100.0
        Force = feedback[4] / 255.0 * 100.0
        print(f"Control result: {Ret}, current state: {State}, position: {Position} / 100, velocity: {Velocity} / 100, force: {Force} / 100.\n")
        
        return Ret, State, Position, Velocity, Force
    
    def open(self):
        '''
        Fully Open the OmniPicker
        '''
        self.control(Pos=100)
    
    def half_open(self, Can_ID=1):
        '''
        Half Open the OmniPicker
        '''
        self.control(Pos=50)
    
    def close(self, Can_ID=1):
        '''
        Fully Close the OmniPicker
        '''
        self.control(Pos=0)
        
if __name__ == "__main__":
    interface = OmniPicker_Interface()
    
    interface.open()
    interface.close()
    interface.half_open()