import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time
import math

# MPU6050 Registers and Address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Initialize I2C bus
        self.bus = smbus2.SMBus(1)  # I2C bus number 1
        self.device_address = MPU6050_ADDR

        # Wake up the MPU6050 since it starts in sleep mode
        self.bus.write_byte_data(self.device_address, PWR_MGMT_1, 0)
        time.sleep(0.1)

        # Publisher to the /imu topic
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)

        # Timer to read and publish data every 0.1 seconds
        self.timer = self.create_timer(0.1, self.read_and_publish_imu_data)

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.device_address, reg)
        low = self.bus.read_byte_data(self.device_address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def get_accel_data(self):
        accel_x = self.read_word_2c(ACCEL_XOUT_H)
        accel_y = self.read_word_2c(ACCEL_XOUT_H + 2)
        accel_z = self.read_word_2c(ACCEL_XOUT_H + 4)
        return accel_x, accel_y, accel_z

    def get_gyro_data(self):
        gyro_x = self.read_word_2c(GYRO_XOUT_H)
        gyro_y = self.read_word_2c(GYRO_XOUT_H + 2)
        gyro_z = self.read_word_2c(GYRO_XOUT_H + 4)
        return gyro_x, gyro_y, gyro_z

    def read_and_publish_imu_data(self):
        accel_x, accel_y, accel_z = self.get_accel_data()
        gyro_x, gyro_y, gyro_z = self.get_gyro_data()

        # Convert raw accelerometer and gyroscope values to standard units
        accel_scale_modifier = 16384.0  # assuming the MPU6050 is set to +/-2g
        gyro_scale_modifier = 131.0     # assuming the MPU6050 is set to +/-250 degrees/s

        accel_x /= accel_scale_modifier
        accel_y /= accel_scale_modifier
        accel_z /= accel_scale_modifier

        gyro_x /= gyro_scale_modifier
        gyro_y /= gyro_scale_modifier
        gyro_z /= gyro_scale_modifier

        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"

        # Linear acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # Angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # Publish the message
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)

    mpu6050_node = MPU6050Node()

    try:
        rclpy.spin(mpu6050_node)
    except KeyboardInterrupt:
        pass
    finally:
        mpu6050_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
