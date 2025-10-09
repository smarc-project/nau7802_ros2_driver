import time
import board
from cedargrove_nau7802 import NAU7802

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

# mostly copied from Adafruit example:
#https://github.com/adafruit/CircuitPython_NAU7802/blob/main/examples/nau7802_simpletest.py

# Instantiate 24-bit load sensor ADC; two channels, default gain of 128
nau7802 = NAU7802(board.I2C(), address=0x2A, active_channels=1)

def zero_channel():
    """Initiate internal calibration for current channel.Use when scale is started,
    a new channel is selected, or to adjust for measurement drift. Remove weight
    and tare from load cell before executing."""
    print(
        "channel {0:1d} calibrate.INTERNAL: {1:5s}".format(
            nau7802.channel, str(nau7802.calibrate("INTERNAL"))
        )
    )
    print(
        "channel {0:1d} calibrate.OFFSET:   {1:5s}".format(
            nau7802.channel, str(nau7802.calibrate("OFFSET"))
        )
    )
    print(f"...channel {nau7802.channel:1d} zeroed")


def read_raw_value(samples=2):
    """Read and average consecutive raw sample values. Return average raw value."""
    sample_sum = 0
    sample_count = samples
    while sample_count > 0:
        while not nau7802.available():
            pass
        sample_sum = sample_sum + nau7802.read()
        sample_count -= 1
    return int(sample_sum / samples)


class NAU7802DriverNode:
    def __init__(self, node: Node, freq: float = 10.0):
        self.node = node

        enabled = nau7802.enable(True)
        self.log("Digital and analog power enabled:", enabled)
        nau7802.channel = 1

        # 42.5k: no load
        # 200k: rope + hook = 790gr
        # 370k: rope + hook + (fake sam + bottle)(0.995gr) = 0.790+0.995 
        self.raw_values = [42500, 200000, 370000]
        self.kg_values = [0.0, 0.790, 0.790+0.995]
        self.log("Calibration values (raw -> grams):", list(zip(self.raw_values, self.kg_values)))

        self.node.declare_parameter('raw_topic', 'sensor/load_cell_raw')
        self.raw_topic = self.node.get_parameter('raw_topic').get_parameter_value().string_value
        self.raw_publisher = node.create_publisher(Int32, self.raw_topic, 10)
        self.raw_msg = Int32()
        self.raw_timer = node.create_timer(1.0 / freq, self.publish_raw)

        self.node.declare_parameter('calibrated_topic', 'sensor/load_cell_weight')
        self.calibrated_topic = self.node.get_parameter('calibrated_topic').get_parameter_value().string_value
        self.calibrated_publisher = node.create_publisher(Float32, self.calibrated_topic, 10)
        self.calibrated_msg = Float32()
        self.calibrated_timer = node.create_timer(1.0 / freq, self.publish_calibrated)

    def publish_calibrated(self):
        raw = read_raw_value()
        for i in range(len(self.raw_values)-1):
            if raw >= self.raw_values[i] and raw <= self.raw_values[i+1]:
                # linear interpolation
                self.calibrated_msg.data = float(self.kg_values[i] + (raw - self.raw_values[i]) * (self.kg_values[i+1] - self.kg_values[i]) / (self.raw_values[i+1] - self.raw_values[i]))
                break
            elif raw < self.raw_values[0]:
                self.calibrated_msg.data = float(self.kg_values[0])
            else:
                # extrapolate from last two points
                self.calibrated_msg.data = float(self.kg_values[-2] + (raw - self.raw_values[-2]) * (self.kg_values[-1] - self.kg_values[-2]) / (self.raw_values[-1] - self.raw_values[-2]))
        self.calibrated_publisher.publish(self.calibrated_msg)
        self.log("Calibrated:", self.calibrated_msg.data, "kg from raw:", raw)

    def publish_raw(self):
        self.raw_msg.data = read_raw_value()
        self.raw_publisher.publish(self.raw_msg)

    def log(self, *args):
        self.node.get_logger().info(" ".join([str(a) for a in args]))

    def exit(self):
        self.log("Exiting, disabling ADC")
        nau7802.enable(False)  # Disable ADC when done
        self.node.destroy_timer(self.raw_timer)
        self.node.destroy_timer(self.calibrated_timer)
        self.log("Ded.")



def __main__():
    rclpy.init()
    node = Node('nau7802_driver_node')
    nau7802_driver_node = NAU7802DriverNode(node)

    rclpy.spin(node)

    nau7802_driver_node.exit()
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    __main__()
