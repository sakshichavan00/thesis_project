import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread

class PoseErrorPlotter(Node):
    def __init__(self):
        super().__init__('pose_error_plotter')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot_pose',
            self.listener_callback,
            10)
        self.positions = []
        self.errors = []
        self.times = []
        self.start_time = None
        
        # Start the live plotting in a separate thread
        self.plot_thread = Thread(target=self.live_plot)
        self.plot_thread.start()

    def listener_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Compute position error using covariance (assuming 2D for simplicity)
        error_x = np.sqrt(msg.pose.covariance[0])  # variance in x
        error_y = np.sqrt(msg.pose.covariance[7])  # variance in y
        error = np.sqrt(error_x**2 + error_y**2)
        
        # Store data for plotting
        self.positions.append((x, y))
        self.errors.append(error)
        self.times.append(time_elapsed)

    def live_plot(self):
        plt.ion()  # Enable interactive mode
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))
        
        while rclpy.ok():
            if self.times:
                axs[0].cla()
                axs[0].set_title('Position Over Time')
                axs[0].set_xlabel('Time [s]')
                axs[0].set_ylabel('Position [m]')
                axs[0].plot(self.times, [p[0] for p in self.positions], label='x')
                axs[0].plot(self.times, [p[1] for p in self.positions], label='y')
                axs[0].legend()

                axs[1].cla()
                axs[1].set_title('Error Over Time')
                axs[1].set_xlabel('Time [s]')
                axs[1].set_ylabel('Error [m]')
                axs[1].plot(self.times, self.errors, label='Error', color='r')
                axs[1].legend()

                plt.pause(0.1)  # Update the plot every 0.1 seconds

        plt.ioff()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    pose_error_plotter = PoseErrorPlotter()
    
    try:
        rclpy.spin(pose_error_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        pose_error_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
