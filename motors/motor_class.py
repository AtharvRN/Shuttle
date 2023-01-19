from pymodbus.client.sync import ModbusSerialClient
import time

class motor:
    
    #register addresses from datasheet
    #Alarm Code - 2 bytes
    Alarm_Code = 0
    #Status Code - 2 bytes
    Status_Code = 2
    #Encoder Position
    Encoder_Position = 10

    ImmediateActualVelocity = 16
    ImmediateTargetVelocity = 17

    ImmediateDriveTemperature = 18
    ImmediateDCBusVoltage = 21
    # 2 bytes
    ImmediatePositionError = 22

    ImmediateCurrentCommand = 30
    ControlMode = 37

    # 2 bytes each
    MaxBrakeDecel = 334
    MaxVelocity = 336
    JogAccel = 338
    JogDecel = 340
    JogVelocity = 342


    CommandOpcode = 124 #7c
    Parameter1 = 125
    Parameter2 = 126
    Parameter3 = 127
    Parameter4 = 128
    Parameter5 = 129

    CommunicationProtocol = 288
    TransmitDelay = 290
    BaudRate = 292
    Address = 294

    TorqueValue = 182
    TorqueLimit = 183

    #opcode values
    motor_enable = 159 #9f
    jog_enable = 162 #a2
    jog_start = 150 #96

    motor_disable = 158 #9e
    jog_disable = 163 #a3
    jog_stop = 216 #d8

    kill_buffer = 225 #0xE1
    # acclerration factor
    a_factor = 6
    # deceleration factor
    d_factor = 240
    # velocity factor
    v_factor = 240
    # wait_time in jogging function
    wait_time = 0.1
    
    client = ModbusSerialClient(method = "rtu", port="/dev/ttyUSB0" ,stopbits = 1, bytesize = 8, parity = 'N', baudrate= 9600)
    def establish_connection():
        motor.client = ModbusSerialClient(method = "rtu", port="/dev/ttyUSB0" ,stopbits = 1, bytesize = 8, parity = 'N', baudrate= 9600)
        connection = motor.client.connect()
        if(connection):
            print("Connection Established")
            return True
        else:
            print("Connection Failure")
            return False

    def __init__(self,accel = 1, decel = 1, velocity = 1, max_velocity = 1, unit = 1) :
        self.velocity = velocity*(motor.v_factor)
        self.accel = accel*(motor.a_factor)
        self.decel = decel*(motor.d_factor)
        self.Unit = unit
        self.max_velocity = max_velocity*(motor.v_factor)
        self.set_params()

    def set_params(self):
        #motor.client.write_registers(address = motor.JogAccel, values = [0,1,0,self.,0,self.velocity], unit = self.Unit)
        motor.client.write_registers(address = motor.JogAccel, values = [0,self.accel,0,self.decel,0,self.velocity], unit = self.Unit )
        #motor.client.write_regsiters(address = motor.MaxVelocity, values = [0,self.max_velocity],unit = self.Unit)
        
    def set_accel(self,accel):
        self.accel = accel
        motor.client.write_registers(address = motor.JogAccel, values = [0,self.accel], unit = self.Unit )

    def set_decel(self,decel):
        self.decel = decel
        motor.client.write_registers(address = motor.JogDecel, values = [0,self.decel], unit = self.Unit )

    def set_velocity(self,velocity):
        self.velocity = velocity
        motor.client.write_registers(address = motor.JogVelocity, values = [0,self.velocity], unit = self.Unit )

    def set_maxVelocity(self,max_velocity):
        self.max_velocity = max_velocity
        max_velocity = motor.client.write_registers(address = motor.MaxVelocity, values = [0,self.max_velocity], unit = self.Unit)

    def get_velocity(self):
        return self.velocity

    def get_accel(self):
        return self.accel

    def get_encoderVelocity(self):
        raw_velocity = motor.client.read_holding_registers(address = motor.ImmediateActualVelocity, count = 1, unit = self.Unit)
        return raw_velocity.registers

    def start_jogging(self):

        # Kill Buffer
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.kill_buffer, unit = self.Unit)
        time.sleep(motor.wait_time)
	
	    #enable motor
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.motor_enable, unit = self.Unit)
        time.sleep(motor.wait_time)
	
	    #enable jogging
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.jog_enable, unit = self.Unit)
        time.sleep(motor.wait_time)
	
	    #start jogging
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.jog_start, unit = self.Unit)
        time.sleep(motor.wait_time)

    def stop_jogging(self):
        
        #stop jogging
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.jog_stop, unit = self.Unit)
        time.sleep(motor.wait_time)

        #disable jogging
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.jog_disable, unit = self.Unit)
        time.sleep(motor.wait_time)

	    #disable motor
        motor.client.write_registers(address = motor.CommandOpcode, values = motor.motor_disable, unit = self.Unit)
        time.sleep(motor.wait_time)

    # ALARM CODES
    def get_alarm_code(self):
        self.alarm_code = motor.client.read_holding_registers(address = motor.Alarm_Code, count = 1 ,unit = self.unit)
        return self.alarm_code
	

if __name__ == "__main__":
    left_motor = motor(unit = 1)
    right_motor = motor(unit = 2)
    motor.establish_connection()
    left_motor.start_jogging()
    right_motor.start_jogging()
    for dog in range(100):
		#Check Velocity
        
	
        print("velocity 1", left_motor.get_encoderVelocity())
        print("velocity 2", right_motor.get_encoderVelocity())


        time.sleep(0.1)
        
        
    left_motor.stop_jogging()
    right_motor.stop_jogging()
    
    for dog in range(100):
		#Check Velocity
        
	
        print("velocity 1", left_motor.get_encoderVelocity())
        print("velocity 2", right_motor.get_encoderVelocity())


        time.sleep(0.1)


