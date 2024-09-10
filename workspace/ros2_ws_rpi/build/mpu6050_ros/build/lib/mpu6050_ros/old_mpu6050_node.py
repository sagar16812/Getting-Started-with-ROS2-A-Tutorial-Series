import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.bus = smbus2.SMBus(1)
        self.device_address = 0x68
        self.init_mpu6050()
        self.timer = self.create_timer(0.1, self.read_and_publish_imu_data)
    
    def init_mpu6050(self):
        self.bus.write_byte_data(self.device_address, 0x6B, 0)  # Wake up the MPU6050

    def read_and_publish_imu_data(self):
        accel_x = self.read_word_2c(0x3B)
        accel_y = self.read_word_2c(0x3D)
        accel_z = self.read_word_2c(0x3F)
        gyro_x = self.read_word_2c(0x43)
        gyro_y = self.read_word_2c(0x45)
        gyro_z = self.read_word_2c(0x47)

        imu_msg = Imu()
        imu_msg.linear_acceleration.x = accel_x / 16384.0  # Scale factor for MPU6050
        imu_msg.linear_acceleration.y = accel_y / 16384.0
        imu_msg.linear_acceleration.z = accel_z / 16384.0
        imu_msg.angular_velocity.x = gyro_x / 131.0  # Scale factor for MPU6050
        imu_msg.angular_velocity.y = gyro_y / 131.0
        imu_msg.angular_velocity.z = gyro_z / 131.0

        self.publisher_.publish(imu_msg)
    
    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.device_address, reg)
        low = self.bus.read_byte_data(self.device_address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

def main(args=None):
    rclpy.init(args=args)
    mpu6050_node = MPU6050Node()
    rclpy.spin(mpu6050_node)
    mpu6050_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
