import time

from adafruit_extended_bus import ExtendedI2C
from cedargrove_nau7802 import NAU7802

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32


class NAU7802DriverNode:
    def __init__(self, node: Node, freq: float = 10.0):
        self.node = node

        self.i2c_bus:int = node.declare_parameter('i2c_bus', 7).get_parameter_value().integer_value
        self.freq:int = node.declare_parameter('frequency', freq).get_parameter_value().double_value
        self.samples:int = node.declare_parameter('samples', 2).get_parameter_value().integer_value
        self.settling_samples:int = node.declare_parameter('settling_samples', 3).get_parameter_value().integer_value
        self.sample_timeout:float = node.declare_parameter('sample_timeout', 1.0).get_parameter_value().double_value


        if self.freq <= 0 or self.samples < 1:
            raise ValueError(f"Invalid frequency or samples: {self.freq}, {self.samples}")

        if self.settling_samples < 0 or self.sample_timeout <= 0:
            raise ValueError(f"Invalid settling or timeout parameter: {self.settling_samples}, {self.sample_timeout}")

        self.i2c = ExtendedI2C(self.i2c_bus)
        self.nau7802 = NAU7802(
            self.i2c,
            address=0x2A, # nau7802 is always on this address, cant be changed!
            active_channels=2
        )

        enabled = self.nau7802.enable(True)

        self.log("I2C bus:", self.i2c_bus)
        self.log("Digital and analog power enabled:", enabled)

        channel1_raw_values = node.declare_parameter('channel_1.raw_values',
                [42500, 200000, 370000]
            ).value
        channel1_kg_values = node.declare_parameter('channel_1.kg_values',
                [0.0, 0.790, 1.785]
            ).value
        
        channel2_raw_values = node.declare_parameter('channel_2.raw_values',
                [0, 0]
            ).value
        channel2_kg_values = node.declare_parameter('channel_2.kg_values',
                [0.0, 0.0]
            ).value
        
        # Independent calibration for each load cell.
        # Channel 2 calibration is disabled until measured.
        self.raw_values = {1:channel1_raw_values, 2:channel2_raw_values}
        self.kg_values = {1:channel1_kg_values, 2:channel2_kg_values}
        

        self.raw_publishers = {}
        self.calibrated_publishers = {}
        self.calibration_valid = {}

        for channel in (1, 2):
            raw_topic = node.declare_parameter(
                f'channel_{channel}.raw_topic',
                f'load_cell_{channel}_raw'
            ).value

            calibrated_topic = node.declare_parameter(
                f'channel_{channel}.calibrated_topic',
                f'load_cell_{channel}_weight'
            ).value

            self.raw_publishers[channel] = node.create_publisher(Int32, raw_topic, 10)
            self.calibrated_publishers[channel] = node.create_publisher(Float32, calibrated_topic, 10)
            self.calibration_valid[channel] = self.validate_calibration(channel)

            if not self.calibration_valid[channel]:
                node.get_logger().warning(f"Channel {channel}: no valid weight calibration. Publishing raw data only.")

            self.log(
                f"Channel {channel} calibration (raw -> kg):",
                list(zip(
                    self.raw_values[channel],
                    self.kg_values[channel])))

        # Select initial channel
        self.nau7802.channel = 1
        self.current_channel = None

        # One timer for both load cells
        self.timer = node.create_timer(1.0 / self.freq, self.publish_measurements)

    def validate_calibration(self, channel):
        raw = self.raw_values[channel]
        kg = self.kg_values[channel]

        return (
            len(raw) >= 2
            and len(raw) == len(kg)
            and all(
                raw[i] < raw[i + 1]
                for i in range(len(raw) - 1)
            )
        )

    def read_sample(self):
        """Wait for and read one ADC sample."""
        deadline = time.monotonic() + self.sample_timeout
        while not self.nau7802.available():
            if time.monotonic() >= deadline:
                raise TimeoutError("NAU7802 conversion timeout")
            time.sleep(0.001)
        return self.nau7802.read()

    def select_channel(self, channel):
        """Select ADC channel and discard settling samples."""
        if self.current_channel == channel:
            return
        self.nau7802.channel = channel
        for _ in range(self.settling_samples):
            self.read_sample()
        self.current_channel = channel

    def read_raw_value(self, channel, samples=None):
        """Read and average samples from one load cell."""
        if samples is None:
            samples = self.samples
        self.select_channel(channel)
        sample_sum = 0
        for _ in range(samples):
            sample_sum += self.read_sample()
        return int(sample_sum / samples)

    def raw_to_kg(self, channel, raw):
        """Convert ADC counts to kg by linear interpolation."""
        raw_values = self.raw_values[channel]
        kg_values = self.kg_values[channel]
        # Limit values below the first calibration point
        if raw <= raw_values[0]:
            return float(kg_values[0])
        # Find interpolation interval
        for i in range(len(raw_values) - 1):
            if raw <= raw_values[i + 1]:
                break
        else:
            # Extrapolate above last calibration point
            i = len(raw_values) - 2

        x0 = raw_values[i]
        x1 = raw_values[i + 1]
        y0 = kg_values[i]
        y1 = kg_values[i + 1]

        return float(y0 + (raw - x0) * (y1 - y0) / (x1 - x0))

    def publish_measurements(self):
        """Read and publish both load cell measurements."""
        for channel in (1, 2):
            try:
                raw = self.read_raw_value(channel)

                # Publish raw ADC value
                raw_msg = Int32()
                raw_msg.data = raw

                self.raw_publishers[channel].publish(raw_msg)

                # Publish calibrated weight
                if self.calibration_valid[channel]:

                    weight = self.raw_to_kg(channel, raw)

                    calibrated_msg = Float32()
                    calibrated_msg.data = weight

                    self.calibrated_publishers[channel].publish(calibrated_msg)

                    self.log(
                        f"CH{channel}:",
                        f"raw={raw}",
                        f"weight={weight:.3f} kg"
                    )

                else:
                    self.log(f"CH{channel}: raw={raw}")

            except (OSError, TimeoutError) as error:
                self.current_channel = None
                self.node.get_logger().error(f"Channel {channel}: {error}")

    def log(self, *args):
        self.node.get_logger().info(" ".join(str(arg) for arg in args))

    def exit(self):
        self.log("Exiting, disabling ADC")
        self.node.destroy_timer(self.timer)
        try:
            self.nau7802.enable(False)
        finally:
            self.i2c.deinit()


def main():
    rclpy.init()

    node = Node('nau7802_driver_node')
    driver = None

    try:
        driver = NAU7802DriverNode(node)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if driver is not None:
                driver.exit()
        finally:
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
