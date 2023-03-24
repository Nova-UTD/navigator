import can
import math
from rosgraph_msgs.msg import Clock


from carla_msgs.msg import CarlaEgoVehicleControl
import rclpy
from rclpy.node import Node
from dataclasses import dataclass


@dataclass
class EpasState():
    duty: int
    current: int
    supply_voltage_mv: int
    temperature: int
    angle: int
    errors: int


class EpasNode(Node):

    def __init__(self):
        super().__init__('epas_node')

        self.limit_left:int = 19
        self.limit_right:int = 230

        self.bus = can.interface.Bus(bustype='slcan', channel='/dev/serial/by-id/usb-Protofusion_Labs_CANable_1205aa6_https:__github.com_normaldotcom_cantact-fw_001500174E50430520303838-if00', bitrate=500000, receive_own_messages=True)
        self.command_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.first_sub, 1)
        self.cmd_msg = None
        self.cmd_timer = self.create_timer(.01,self.vehicleControlCb)
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.cached_msg1 = None

    def first_sub(self,msg: CarlaEgoVehicleControl):
        self.cmd_msg = msg

    def parseIncomingMessages(self, msg1_data: bytearray, msg2_data: bytearray) -> EpasState:
        torque = msg1_data[0]
        duty = msg1_data[1]
        current = msg1_data[2]
        supply_voltage = msg1_data[3]
        switch_pos = msg1_data[4]
        temp = msg1_data[5]
        torque_a = msg1_data[6]
        torque_b = msg1_data[7]

        angle = msg2_data[0]
        analog_c1 = msg2_data[1]
        analog_c2 = msg2_data[2]
        selected_map = msg2_data[3]
        errors = msg2_data[4]
        dio_bitfield = msg2_data[5]
        status_bitfield = msg2_data[6]
        limit_bitfield = msg2_data[7]

        current_state = EpasState(
            duty, current, supply_voltage, temp, angle, errors)

        return current_state

    def sendCommand(self, target, bus):
        current_angle_normalized = ((self.current_angle-self.limit_left)/(self.limit_right-self.limit_left)*2)-1
        #self.get_logger().info('current angle'+str(current_angle_normalized))
        e = target - current_angle_normalized # Error = target - current
        self.get_logger().info('current_angle_normalized: '+str(current_angle_normalized))
        self.get_logger().info('target: '+str(target))

        # We need to map [-1.0, 1.0] to [0, 255]

        Kp = 1

        power = e * Kp # Power is an abstract value from [-1., 1.], where -1 is hard push left

        torqueA: int = min(255, max(0, math.ceil((power+1) * (255/2))))
        torqueB: int = 255-torqueA

        print (f"{torqueA}")
        #self.get_logger().info(str(torqueA))

        data = [0x03, torqueA, torqueB, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x296, data=data, check=True, is_extended_id=False)
        bus.send(message, timeout=0.2)

        print (f"{torqueA}")
        # print(e)

    def vehicleControlCb(self):
        if self.cmd_msg!=None:
            self.target_angle = self.cmd_msg.steer
        #self.get_logger().info(str(msg.steer))
        

        response_msg = self.bus.recv(0.1)
        if (response_msg is None):
            print("No message received")
        elif (response_msg.arbitration_id == 0x290):
            self.cached_msg1 = response_msg.data
        elif (response_msg.arbitration_id == 0x292):
            if (self.cached_msg1 is None):
                return
            msg1 = self.cached_msg1
            msg2 = response_msg.data
            current_state = self.parseIncomingMessages(msg1, msg2)
            self.current_angle = current_state.angle
            
            #self.get_logger().info(str(self.current_angle))

        # self.get_logger().info(f"Target: {self.target_angle}, current: {self.current_angle}")

        self.sendCommand(self.target_angle, self.bus)

        # current_angle_norm = (
        #     (self.current_angle-self.limit_left)/(self.limit_right-self.limit_left)*2)-1
        # self.error = self.target_angle-current_angle_norm
        # self.torque_a = int(
        #     min(255, max(0, math.ceil((self.error+1) * (255/2)))))
        # self.torque_b = 255-self.torque_a

        # data = [0x03, self.torque_a, self.torque_b,
        #         0x00, 0x00, 0x00, 0x00, 0x00]
        # message = can.Message(arbitration_id=0x296, data=data,
        #                       check=True, is_extended_id=False)
        # self.bus.send(message, timeout=0.2)

        # The EPAS sends response info in two halves, each its own
        # kind of CAN message. Combining the two gives us a complete state
        # of the EPAS. Let's combine the two now.
        



def main(args=None):
    rclpy.init(args=args)
    epas_node = EpasNode()
    rclpy.spin(epas_node)
    epas_node.destroy_node()
    rclpy.shutdown()
