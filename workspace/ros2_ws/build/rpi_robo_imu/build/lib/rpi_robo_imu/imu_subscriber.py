import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import csv
import os
import sys

class ImuSubscriber(Node):
    def __init__(self, save_data):
        super().__init__('imu_subscriber_node')

        # Subscription to the /imu topic
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Subscription to the /temperature topic
        self.temp_subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_listener_callback,
            10
        )
        self.temp_subscription  # prevent unused variable warning

        # Temperature data storage
        self.current_temperature = None

        # Saving data settings
        self.save_data = save_data
        self.file = None
        self.csv_writer = None

        # If save_data is True, open the file in append mode
        if self.save_data:
            self.file = open('imu_data.csv', 'a', newline='')
            self.csv_writer = csv.writer(self.file)

            # Check if the file is empty (write headers only if it's a new file)
            if os.stat('imu_data.csv').st_size == 0:
                self.csv_writer.writerow(['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Temperature'])
            self.get_logger().info('Started saving IMU and temperature data to imu_data.csv.')

    def imu_listener_callback(self, msg):
        # Extract accelerometer and gyroscope data from the message
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Print IMU data to the console
        self.get_logger().info(f'Accel: x={accel_x:.3f}, y={accel_y:.3f}, z={accel_z:.3f} | '
                               f'Gyro: x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}')

        # If temperature data is available, print it
        if self.current_temperature is not None:
            self.get_logger().info(f'Temperature: {self.current_temperature:.2f} °C')

        # Append IMU and temperature data to CSV file if saving is enabled
        if self.save_data and self.csv_writer:
            self.csv_writer.writerow([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, self.current_temperature])

    def temperature_listener_callback(self, msg):
        # Update the current temperature from the temperature message
        # self.get_logger().info(f"Temperature callback triggered. Data: {msg.data}")  # Debugging line
        self.current_temperature = msg.data

    def close_file(self):
        # Close the file if it's open
        if self.file:
            self.file.close()
            self.get_logger().info('IMU data file closed.')

def main(args=None):
    rclpy.init(args=args)

    try:
        # Ask the user at the beginning if they want to save data
        save_data_input = input("Do you want to save IMU and temperature data to a CSV file? (y/n): ").strip().lower()
        save_data = save_data_input == 'y'

        # Initialize the subscriber node with the save_data option
        imu_subscriber = ImuSubscriber(save_data=save_data)

        # Keep spinning and listening for IMU and temperature data
        rclpy.spin(imu_subscriber)

    except KeyboardInterrupt:
        # Safely exit on Ctrl+C
        imu_subscriber.get_logger().info("Interrupted! Shutting down safely...")

    finally:
        # Make sure the file is properly closed
        imu_subscriber.close_file()

        # Shutdown ROS only if it's not already shut down
        if rclpy.ok():
            imu_subscriber.get_logger().info("Shutting down ROS...")
            imu_subscriber.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
