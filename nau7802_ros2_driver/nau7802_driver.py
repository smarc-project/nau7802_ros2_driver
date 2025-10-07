import time
import board
from cedargrove_nau7802 import NAU7802

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

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
        self.publisher = node.create_publisher(Int32, 'sensor/load_cell/raw', 10)
        self.msg = Int32()
        self.timer = node.create_timer(1.0 / freq, self.publish)

    def publish(self):
        self.msg.data = read_raw_value()
        self.publisher.publish(self.msg)
        self.node.get_logger().info(f'Publishing: {self.msg.data}')




def __main__():
    # print("Calibrating loadcell, please wait...")
    enabled = nau7802.enable(True)
    print("Digital and analog power enabled:", enabled)
    # print("REMOVE WEIGHTS FROM LOAD CELLS IN 3 SECONDS")
    # time.sleep(3)

    nau7802.channel = 1
    # zero_channel()  # Calibrate and zero channel

    print("Publishing...")

    rclpy.init()
    node = Node('nau7802_driver_node')
    nau7802_driver_node = NAU7802DriverNode(node)

    rclpy.spin(node)
    
    nau7802.enable(False)  # Disable ADC when done

    node.destroy_timer(nau7802_driver_node.timer)
    node.destroy_node()
    rclpy.shutdown()

    # 42.5k: no load, no zeroing
    # 200k: rope + hook
    # 370k: rope + hook + fake sam + bottle


if __name__ == '__main__':
    __main__()