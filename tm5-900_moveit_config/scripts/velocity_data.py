#!/usr/bin/env python3
"""
TM Robot PVT Velocity Test - Ultra Smooth Version
Adjustable smoothing levels for perfectly smooth curves
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tm_msgs.msg import FeedbackState
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter, butter, filtfilt
from scipy.ndimage import gaussian_filter1d
from datetime import datetime

class UltraSmoothVelocityTest(Node):
    def __init__(self, test_duration=10.0, target_velocity=0.01, smoothing_level='ultra'):
        """
        smoothing_level: 'light', 'medium', 'strong', 'ultra', 'extreme'
        """
        super().__init__('ultra_smooth_velocity_test')
        
        self.test_duration = test_duration
        self.target_velocity_mm = target_velocity * 1000.0
        self.smoothing_level = smoothing_level
        
        self.timestamps = []
        self.positions = []
        self.start_time = None
        self.test_active = False
        
        # ROS2 interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_sub = self.create_subscription(
            FeedbackState, '/feedback_states', self.feedback_callback, 10)
        self.cmd_timer = self.create_timer(0.1, self.send_velocity_command)
        
        level_names = {
            'light': 'Light (preserves detail)',
            'medium': 'Medium (balanced)',
            'strong': 'Strong (very clean)',
            'ultra': 'Ultra (extremely smooth)',
            'extreme': 'Extreme (maximum smoothing)'
        }
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'🎯 Test: {self.target_velocity_mm:.1f} mm/s for {test_duration:.1f}s')
        self.get_logger().info(f'📊 Smoothing: {level_names.get(smoothing_level, smoothing_level)}')
        self.get_logger().info('='*60)
        
    def feedback_callback(self, msg):
        if not self.test_active:
            return
            
        elapsed = time.time() - self.start_time
        x_mm = msg.tool_pose[0] * 1000.0
        
        self.timestamps.append(elapsed)
        self.positions.append(x_mm)
        
        if elapsed >= self.test_duration:
            self.get_logger().info(f'✅ Test completed! ({len(self.positions)} points)')
            self.stop_test()
            
    def send_velocity_command(self):
        if not self.test_active:
            if self.start_time is None:
                self.start_time = time.time()
                self.test_active = True
                self.get_logger().info('🚀 Test started!')
            return
            
        msg = Twist()
        msg.linear.x = self.target_velocity_mm / 1000.0
        self.cmd_vel_pub.publish(msg)
        
    def stop_test(self):
        self.test_active = False
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('🔧 Applying ultra-smooth filters...')
        self.generate_plot()
        rclpy.shutdown()
        
    def apply_butterworth_filter(self, data, cutoff_freq=0.5, order=4):
        """
        Butterworth low-pass filter - extremely smooth
        cutoff_freq: normalized frequency (0-1), lower = smoother
        """
        if len(data) < 10:
            return data
            
        b, a = butter(order, cutoff_freq, btype='low')
        filtered = filtfilt(b, a, data)
        return filtered
        
    def apply_gaussian_filter(self, data, sigma=5):
        """
        Gaussian filter - very smooth
        sigma: smoothing strength, higher = smoother
        """
        if len(data) < 3:
            return data
        return gaussian_filter1d(data, sigma=sigma)
        
    def ultra_smooth(self, velocities):
        """Apply ultra-smooth filtering based on level"""
        velocities = np.array(velocities)
        
        if len(velocities) < 10:
            return velocities
        
        if self.smoothing_level == 'light':
            # Savitzky-Golay only
            window = min(31, len(velocities) if len(velocities) % 2 == 1 else len(velocities) - 1)
            if window < 5:
                window = 5
            smoothed = savgol_filter(velocities, window, 3)
            
        elif self.smoothing_level == 'medium':
            # Gaussian filter (moderate)
            smoothed = self.apply_gaussian_filter(velocities, sigma=3)
            
        elif self.smoothing_level == 'strong':
            # Gaussian + Savitzky-Golay
            temp = self.apply_gaussian_filter(velocities, sigma=5)
            window = min(41, len(temp) if len(temp) % 2 == 1 else len(temp) - 1)
            if window < 5:
                window = 5
            smoothed = savgol_filter(temp, window, 3)
            
        elif self.smoothing_level == 'ultra':
            # Butterworth filter (low cutoff)
            smoothed = self.apply_butterworth_filter(velocities, cutoff_freq=0.3, order=4)
            # Additional Gaussian smoothing
            smoothed = self.apply_gaussian_filter(smoothed, sigma=3)
            
        elif self.smoothing_level == 'extreme':
            # Multiple passes for maximum smoothness
            smoothed = self.apply_butterworth_filter(velocities, cutoff_freq=0.2, order=5)
            smoothed = self.apply_gaussian_filter(smoothed, sigma=8)
            window = min(51, len(smoothed) if len(smoothed) % 2 == 1 else len(smoothed) - 1)
            if window < 5:
                window = 5
            smoothed = savgol_filter(smoothed, window, 3)
            
        else:
            smoothed = velocities
            
        return smoothed
        
    def generate_plot(self):
        if len(self.positions) < 2:
            self.get_logger().error('Not enough data!')
            return
            
        # Calculate raw velocities
        velocities_raw = []
        vel_times = []
        for i in range(1, len(self.positions)):
            dt = self.timestamps[i] - self.timestamps[i-1]
            dx = self.positions[i] - self.positions[i-1]
            if dt > 0:
                velocities_raw.append(dx / dt)
                vel_times.append(self.timestamps[i])
        
        # Apply ultra-smooth filtering
        velocities_smooth = self.ultra_smooth(velocities_raw)
        
        # Create figure with 2 main plots
        fig = plt.figure(figsize=(14, 10))
        gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 0.3], hspace=0.35, wspace=0.25)
        
        # ========== Top: Velocity Comparison ==========
        ax1 = fig.add_subplot(gs[0, :])
        
        # Plot raw (very faint background)
        ax1.plot(vel_times, velocities_raw, color='lightgray', linewidth=1, 
                alpha=0.5, label='Raw data', zorder=1)
        
        # Plot smoothed (prominent)
        ax1.plot(vel_times, velocities_smooth, 'b-', linewidth=3, 
                label=f'Smoothed ({self.smoothing_level} level)', zorder=3)
        
        # Target line
        ax1.axhline(self.target_velocity_mm, color='r', linestyle='--', 
                   linewidth=2.5, label=f'Target: {self.target_velocity_mm:.1f} mm/s', zorder=2)
        
        # Find and mark stable region
        lower, upper = self.target_velocity_mm * 0.85, self.target_velocity_mm * 1.15
        stable_idx = [i for i, v in enumerate(velocities_smooth) if lower < v < upper]
        
        if stable_idx:
            t_start, t_end = vel_times[stable_idx[0]], vel_times[stable_idx[-1]]
            ax1.axvspan(t_start, t_end, alpha=0.2, color='green', 
                       label='Stable Phase', zorder=0)
            
            stable_vels = [velocities_smooth[i] for i in stable_idx]
            avg_stable = np.mean(stable_vels)
            mid_time = (t_start + t_end) / 2
            
            ax1.text(mid_time, avg_stable + 2.5,
                    f'Stable Phase\n{avg_stable:.2f} mm/s ({avg_stable/self.target_velocity_mm*100:.1f}%)',
                    ha='center', fontsize=11, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', 
                             edgecolor='green', linewidth=2, alpha=0.9))
        
        # Annotate motion phases
        n = len(velocities_smooth)
        phase_colors = ['#FFE6CC', '#FFD9B3', '#E8F5E9', '#FFD9B3', '#E0E0E0']
        phase_labels = ['Startup', 'Acceleration', 'Stable Motion', 'Deceleration', 'Stop']
        phase_bounds = [0, n//5, 2*n//5, 3*n//5, 4*n//5, n]
        
        for i in range(5):
            start_idx = phase_bounds[i]
            end_idx = phase_bounds[i+1]
            if start_idx < len(vel_times) and end_idx <= len(vel_times):
                ax1.axvspan(vel_times[start_idx], 
                          vel_times[min(end_idx-1, len(vel_times)-1)],
                          alpha=0.1, color=phase_colors[i], zorder=0)
                
                mid_idx = (start_idx + end_idx) // 2
                if mid_idx < len(vel_times):
                    y_pos = max(velocities_smooth) * 0.92
                    ax1.text(vel_times[mid_idx], y_pos, phase_labels[i],
                           ha='center', fontsize=9, style='italic', 
                           color='gray', alpha=0.7)
        
        ax1.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Velocity (mm/s)', fontsize=12, fontweight='bold')
        ax1.set_title('(a) Velocity vs Time - Ultra Smooth Analysis', 
                     fontsize=13, fontweight='bold', pad=10)
        ax1.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
        ax1.legend(fontsize=10, loc='upper right', framealpha=0.95, 
                  edgecolor='black', fancybox=True)
        ax1.set_ylim([min(velocities_smooth)*0.95, max(velocities_smooth)*1.05])
        
        # ========== Middle Left: Position ==========
        ax2 = fig.add_subplot(gs[1, :])
        ax2.plot(self.timestamps, self.positions, 'g-', linewidth=2.5, 
                label='Actual Position', marker='o', markersize=2, markevery=10)
        
        theoretical = [self.positions[0] + self.target_velocity_mm * t 
                      for t in self.timestamps]
        ax2.plot(self.timestamps, theoretical, 'r--', linewidth=2, 
                label='Theoretical (constant velocity)', alpha=0.7)
        
        ax2.fill_between(self.timestamps, self.positions, theoretical, 
                        alpha=0.2, color='orange')
        
        ax2.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
        ax2.set_ylabel('Position X (mm)', fontsize=11, fontweight='bold')
        ax2.set_title('(b) Position Trajectory', fontsize=12, fontweight='bold', pad=8)
        ax2.grid(True, alpha=0.3, linestyle='--')
        ax2.legend(fontsize=9, loc='best', framealpha=0.9)
        
        # ========== Middle Right: Velocity Distribution ==========
        # ax3 = fig.add_subplot(gs[1, 1])
        
        # counts, bins, patches = ax3.hist(velocities_smooth, bins=40, 
        #                                  color='skyblue', edgecolor='navy', 
        #                                  alpha=0.7, linewidth=1.5)
        
        # # Color code histogram bars
        # for i, patch in enumerate(patches):
        #     bin_center = (bins[i] + bins[i+1]) / 2
        #     if lower <= bin_center <= upper:
        #         patch.set_facecolor('lightgreen')
        #         patch.set_edgecolor('green')
        
        # ax3.axvline(self.target_velocity_mm, color='r', linestyle='--', 
        #            linewidth=3, label='Target Velocity')
        # ax3.axvline(np.mean(velocities_smooth), color='blue', linestyle='-', 
        #            linewidth=2.5, label='Mean Velocity', alpha=0.8)
        
        # ax3.set_xlabel('Velocity (mm/s)', fontsize=11, fontweight='bold')
        # ax3.set_ylabel('Frequency', fontsize=11, fontweight='bold')
        # ax3.set_title('(c) Velocity Distribution', fontsize=12, fontweight='bold', pad=8)
        # ax3.legend(fontsize=9, loc='best', framealpha=0.9)
        # ax3.grid(True, alpha=0.3, axis='y')
        
        # ========== Bottom: Statistics Table ==========
        ax4 = fig.add_subplot(gs[2, :])
        ax4.axis('off')
        
        # Calculate comprehensive statistics
        displacement = self.positions[-1] - self.positions[0]
        avg_vel = np.mean(velocities_smooth)
        stable_vel = np.mean([velocities_smooth[i] for i in stable_idx]) if stable_idx else 0
        max_vel = np.max(velocities_smooth)
        min_vel = np.min(velocities_smooth)
        std_vel = np.std(velocities_smooth)
        stable_ratio = len(stable_idx) / len(velocities_smooth) * 100 if stable_idx else 0
        
        # Create data for table
        table_data = [
            ['Parameter', 'Value', 'Achievement', '│', 'Motion Profile', 'Value'],
            ['─'*20, '─'*15, '─'*12, '│', '─'*20, '─'*15],
            ['Target Velocity', f'{self.target_velocity_mm:.2f} mm/s', '100.0%',
             '│', 'Total Displacement', f'{displacement:.2f} mm'],
            ['Average Velocity', f'{avg_vel:.2f} mm/s', f'{avg_vel/self.target_velocity_mm*100:.1f}%',
             '│', 'Test Duration', f'{self.timestamps[-1]:.2f} s'],
            ['Stable Phase Avg', f'{stable_vel:.2f} mm/s', f'{stable_vel/self.target_velocity_mm*100:.1f}%',
             '│', 'Data Points', f'{len(self.positions)}'],
            ['Max Velocity', f'{max_vel:.2f} mm/s', f'{max_vel/self.target_velocity_mm*100:.1f}%',
             '│', 'Stable Phase Ratio', f'{stable_ratio:.1f}%'],
            ['Velocity Std Dev', f'{std_vel:.2f} mm/s', f'{std_vel/self.target_velocity_mm*100:.1f}%',
             '│', 'Smoothing Level', self.smoothing_level.upper()],
        ]
        
        # Create table
        # table = ax4.table(cellText=table_data, cellLoc='center', loc='center',
        #                  colWidths=[0.18, 0.14, 0.11, 0.02, 0.18, 0.14])
        # table.auto_set_font_size(False)
        # table.set_fontsize(9)
        # table.scale(1, 2.2)
        
        # Style the table
        # for i in range(len(table_data)):
        #     for j in range(len(table_data[0])):
        #         cell = table[(i, j)]
        #         if i == 0:  # Header
        #             cell.set_facecolor('#2E7D32')
        #             cell.set_text_props(weight='bold', color='white', fontsize=10)
        #         elif i == 1:  # Separator
        #             cell.set_facecolor('#E8F5E9')
        #             cell.set_text_props(color='gray')
        #         elif j == 3:  # Divider column
        #             cell.set_facecolor('white')
        #         else:
        #             if i % 2 == 0:
        #                 cell.set_facecolor('#F5F5F5')
        #             else:
        #                 cell.set_facecolor('white')
                        
        #         # Highlight stable phase row
        #         if i == 4:
        #             cell.set_facecolor('#C8E6C9')
        
        # Overall title
        smoothing_desc = {
            'light': 'Light Smoothing',
            'medium': 'Medium Smoothing',
            'strong': 'Strong Smoothing',
            'ultra': 'Ultra Smoothing',
            'extreme': 'Extreme Smoothing'
        }
        
        fig.suptitle(f'TM Robot PVT Velocity Control - {smoothing_desc.get(self.smoothing_level, "Custom")}', 
                    fontsize=15, fontweight='bold', y=0.98)
        
        # Save
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'pvt_velocity_{self.smoothing_level}_{timestamp}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight', facecolor='white')
        
        # Print summary
        print(f'\n{"="*70}')
        print(f'✅ ULTRA-SMOOTH PLOT SAVED: {filename}')
        print(f'{"="*70}')
        print(f'Smoothing Level:      {self.smoothing_level.upper()}')
        print(f'Target Velocity:      {self.target_velocity_mm:.2f} mm/s')
        print(f'Average Velocity:     {avg_vel:.2f} mm/s ({avg_vel/self.target_velocity_mm*100:.1f}%)')
        print(f'Stable Phase Avg:     {stable_vel:.2f} mm/s ({stable_vel/self.target_velocity_mm*100:.1f}%)')
        print(f'Velocity Std Dev:     {std_vel:.2f} mm/s ({std_vel/self.target_velocity_mm*100:.1f}%)')
        print(f'Stable Phase Ratio:   {stable_ratio:.1f}%')
        print(f'Displacement:         {displacement:.2f} mm')
        print(f'Test Duration:        {self.timestamps[-1]:.2f} s')
        print(f'{"="*70}\n')
        
        plt.close()

def main():
    rclpy.init()
    
    # ===== CONFIGURATION =====
    TEST_DURATION = 3.0       # seconds
    TARGET_VELOCITY = 0.01     # m/s (= 10 mm/s)
    
    # Choose smoothing level:
    # 'light'   - Minimal smoothing, preserves detail
    # 'medium'  - Balanced smoothing
    # 'strong'  - Very smooth curves
    # 'ultra'   - Extremely smooth (recommended) ⭐
    # 'extreme' - Maximum smoothing (magazine quality)
    
    SMOOTHING_LEVEL = 'ultra'  # <-- Change this for different smoothness
    
    # =========================
    
    print('\n' + '='*70)
    print('TM Robot Ultra-Smooth Velocity Test')
    print('='*70)
    print(f'Target: {TARGET_VELOCITY*1000:.1f} mm/s | Duration: {TEST_DURATION:.1f}s')
    print(f'Smoothing: {SMOOTHING_LEVEL.upper()}')
    print('='*70 + '\n')
    
    node = UltraSmoothVelocityTest(TEST_DURATION, TARGET_VELOCITY, SMOOTHING_LEVEL)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⚠️  Test interrupted!')
        if len(node.positions) > 1:
            node.generate_plot()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
# #!/usr/bin/env python3
# """
# Simplified TM Robot PVT Velocity Test
# Generates a single comprehensive plot
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from tm_msgs.msg import FeedbackState
# import time
# import matplotlib.pyplot as plt
# import numpy as np
# from datetime import datetime

# class SimpleVelocityTest(Node):
#     def __init__(self, test_duration=3.0, target_velocity=0.01):
#         super().__init__('simple_velocity_test')
        
#         self.test_duration = test_duration
#         self.target_velocity_mm = target_velocity * 1000.0  # Convert to mm/s
        
#         self.timestamps = []
#         self.positions = []
#         self.start_time = None
#         self.test_active = False
        
#         # ROS2 interfaces
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.feedback_sub = self.create_subscription(
#             FeedbackState, '/feedback_states', self.feedback_callback, 10)
#         self.cmd_timer = self.create_timer(0.1, self.send_velocity_command)
        
#         self.get_logger().info(f'Test: {self.target_velocity_mm:.1f} mm/s for {test_duration:.1f}s')
        
#     def feedback_callback(self, msg):
#         if not self.test_active:
#             return
            
#         elapsed = time.time() - self.start_time
#         x_mm = msg.tool_pose[0] * 1000.0
        
#         self.timestamps.append(elapsed)
#         self.positions.append(x_mm)
        
#         if elapsed >= self.test_duration:
#             self.get_logger().info(f'Test completed! ({len(self.positions)} points)')
#             self.stop_test()
            
#     def send_velocity_command(self):
#         if not self.test_active:
#             if self.start_time is None:
#                 self.start_time = time.time()
#                 self.test_active = True
#                 self.get_logger().info('🚀 Test started!')
#             return
            
#         msg = Twist()
#         msg.linear.x = self.target_velocity_mm / 1000.0  # Convert back to m/s
#         self.cmd_vel_pub.publish(msg)
        
#     def stop_test(self):
#         self.test_active = False
#         self.cmd_vel_pub.publish(Twist())  # Stop robot
#         self.generate_plot()
#         rclpy.shutdown()
        
#     def generate_plot(self):
#         if len(self.positions) < 2:
#             return
            
#         # Calculate velocities
#         velocities = []
#         vel_times = []
#         for i in range(1, len(self.positions)):
#             dt = self.timestamps[i] - self.timestamps[i-1]
#             dx = self.positions[i] - self.positions[i-1]
#             if dt > 0:
#                 velocities.append(dx / dt)
#                 vel_times.append(self.timestamps[i])
        
#         # Create plot
#         fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9))
        
#         # Velocity plot
#         ax1.plot(vel_times, velocities, 'b-', linewidth=2, label='Actual Velocity')
#         ax1.axhline(self.target_velocity_mm, color='r', linestyle='--', 
#                    linewidth=2, label=f'Target: {self.target_velocity_mm:.1f} mm/s')
        
#         # Find and mark stable region
#         lower, upper = self.target_velocity_mm * 0.8, self.target_velocity_mm * 1.2
#         stable_idx = [i for i, v in enumerate(velocities) if lower < v < upper]
        
#         if stable_idx:
#             t_start, t_end = vel_times[stable_idx[0]], vel_times[stable_idx[-1]]
#             ax1.axvspan(t_start, t_end, alpha=0.2, color='green', label='Stable Phase')
            
#             stable_vels = [velocities[i] for i in stable_idx]
#             avg_stable = np.mean(stable_vels)
#             ax1.text((t_start + t_end) / 2, avg_stable + 1.5,
#                     f'Stable: {avg_stable:.2f} mm/s\n({avg_stable/self.target_velocity_mm*100:.1f}%)',
#                     ha='center', fontsize=10, 
#                     bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        
#         ax1.set_xlabel('Time (s)', fontsize=12)
#         ax1.set_ylabel('Velocity (mm/s)', fontsize=12)
#         ax1.set_title('(a) Velocity vs Time', fontsize=13, fontweight='bold')
#         ax1.grid(True, alpha=0.3, linestyle='--')
#         ax1.legend(fontsize=10, loc='best')
        
#         # Position plot
#         ax2.plot(self.timestamps, self.positions, 'g-', linewidth=2, label='Actual Position')
        
#         # Theoretical line
#         theoretical = [self.positions[0] + self.target_velocity_mm * t 
#                       for t in self.timestamps]
#         ax2.plot(self.timestamps, theoretical, 'r--', linewidth=2, 
#                 label='Theoretical (constant velocity)', alpha=0.6)
        
#         ax2.set_xlabel('Time (s)', fontsize=12)
#         ax2.set_ylabel('Position X (mm)', fontsize=12)
#         ax2.set_title('(b) Position vs Time', fontsize=13, fontweight='bold')
#         ax2.grid(True, alpha=0.3, linestyle='--')
#         ax2.legend(fontsize=10, loc='best')
        
#         # Add statistics text box
#         displacement = self.positions[-1] - self.positions[0]
#         avg_vel = np.mean(velocities)
#         stable_vel = np.mean([velocities[i] for i in stable_idx]) if stable_idx else 0
        
#         stats_text = (f'Target: {self.target_velocity_mm:.1f} mm/s\n'
#                      f'Avg Velocity: {avg_vel:.2f} mm/s ({avg_vel/self.target_velocity_mm*100:.1f}%)\n'
#                      f'Stable Phase: {stable_vel:.2f} mm/s ({stable_vel/self.target_velocity_mm*100:.1f}%)\n'
#                      f'Displacement: {displacement:.2f} mm\n'
#                      f'Duration: {self.timestamps[-1]:.2f} s')
        
#         fig.text(0.98, 0.02, stats_text, fontsize=9, 
#                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
#                 ha='right', va='bottom', family='monospace')
        
#         plt.suptitle('TM Robot PVT Velocity Control Test Results', 
#                     fontsize=14, fontweight='bold', y=0.995)
#         plt.tight_layout(rect=[0, 0.05, 1, 0.99])
        
#         # Save
#         timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
#         filename = f'pvt_velocity_test_{timestamp}.png'
#         plt.savefig(filename, dpi=300, bbox_inches='tight')
        
#         print(f'\n✅ Plot saved: {filename}')
#         print(f'\n📊 Results Summary:')
#         print(f'   Target Velocity:  {self.target_velocity_mm:.2f} mm/s')
#         print(f'   Average Velocity: {avg_vel:.2f} mm/s ({avg_vel/self.target_velocity_mm*100:.1f}%)')
#         print(f'   Stable Velocity:  {stable_vel:.2f} mm/s ({stable_vel/self.target_velocity_mm*100:.1f}%)')
#         print(f'   Displacement:     {displacement:.2f} mm')
#         print(f'   Test Duration:    {self.timestamps[-1]:.2f} s\n')
        
#         plt.close()

# def main():
#     rclpy.init()
    
#     # You can modify these parameters
#     TEST_DURATION = 3.0    # seconds
#     TARGET_VELOCITY = -0.01  # m/s (= 10 mm/s)
    
#     node = SimpleVelocityTest(TEST_DURATION, TARGET_VELOCITY)
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n⚠️  Test interrupted!')
#         if len(node.positions) > 1:
#             node.generate_plot()
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# #!/usr/bin/env python3
# """
# TM Robot PVT Velocity Control Test with Real-time Data Collection
# Simultaneously sends velocity commands and collects feedback data
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from tm_msgs.msg import FeedbackState
# import time
# import matplotlib.pyplot as plt
# import numpy as np
# from datetime import datetime

# class VelocityTestNode(Node):
#     def __init__(self, test_duration=3.0, target_velocity=0.01):
#         super().__init__('velocity_test_node')
        
#         # Test parameters
#         self.test_duration = test_duration  # seconds
#         self.target_velocity = target_velocity  # m/s
#         self.target_velocity_mm = target_velocity * 1000.0  # mm/s
        
#         # Data storage
#         self.timestamps = []
#         self.positions = []
#         self.velocities = []
#         self.tcp_speeds = []
        
#         self.start_time = None
#         self.test_active = False
#         self.initial_position = None
        
#         # Publishers and Subscribers
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.feedback_sub = self.create_subscription(
#             FeedbackState,
#             '/feedback_states',
#             self.feedback_callback,
#             10
#         )
        
#         # Timer for sending velocity commands (10 Hz)
#         self.cmd_timer = self.create_timer(0.1, self.send_velocity_command)
        
#         self.get_logger().info('='*60)
#         self.get_logger().info('TM Robot PVT Velocity Test')
#         self.get_logger().info(f'Target Velocity: {self.target_velocity_mm:.2f} mm/s')
#         self.get_logger().info(f'Test Duration: {self.test_duration:.1f} seconds')
#         self.get_logger().info('='*60)
#         self.get_logger().info('Waiting for initial feedback...')
        
#     def feedback_callback(self, msg):
#         """Collect feedback data"""
#         if not self.test_active:
#             return
            
#         current_time = time.time()
#         elapsed = current_time - self.start_time
        
#         # Extract position (convert to mm)
#         x_mm = msg.tool_pose[0] * 1000.0
        
#         # Store data
#         self.timestamps.append(elapsed)
#         self.positions.append(x_mm)
        
#         # Calculate instantaneous velocity
#         if len(self.positions) > 1:
#             dt = self.timestamps[-1] - self.timestamps[-2]
#             dx = self.positions[-1] - self.positions[-2]
#             if dt > 0:
#                 vel = dx / dt
#                 self.velocities.append(vel)
        
#         # Store TCP speed from feedback
#         tcp_speed_x = msg.tcp_speed[0] * 1000.0  # Convert to mm/s
#         self.tcp_speeds.append(tcp_speed_x)
        
#         # Check if test is complete
#         if elapsed >= self.test_duration:
#             self.get_logger().info(f'\nTest completed! ({elapsed:.2f} seconds)')
#             self.get_logger().info(f'Data points collected: {len(self.positions)}')
#             self.stop_test()
            
#     def send_velocity_command(self):
#         """Send velocity command at 10 Hz"""
#         if not self.test_active:
#             if self.start_time is None:
#                 # Start test on first call
#                 self.start_test()
#             return
            
#         # Create and publish Twist message
#         msg = Twist()
#         msg.linear.x = self.target_velocity
#         msg.linear.y = 0.0
#         msg.linear.z = 0.0
#         msg.angular.x = 0.0
#         msg.angular.y = 0.0
#         msg.angular.z = 0.0
        
#         self.cmd_vel_pub.publish(msg)
        
#     def start_test(self):
#         """Start the test"""
#         self.start_time = time.time()
#         self.test_active = True
#         self.get_logger().info('\n🚀 Test started!')
#         self.get_logger().info('Collecting data...\n')
        
#     def stop_test(self):
#         """Stop the test"""
#         self.test_active = False
        
#         # Send zero velocity to stop robot
#         stop_msg = Twist()
#         self.cmd_vel_pub.publish(stop_msg)
        
#         self.get_logger().info('Robot stopped.')
#         self.get_logger().info('Generating plots...\n')
        
#         # Generate plots
#         self.generate_plots()
        
#         # Shutdown node
#         self.get_logger().info('Shutting down...')
#         rclpy.shutdown()
        
#     def generate_plots(self):
#         """Generate analysis plots"""
#         if len(self.positions) < 2:
#             self.get_logger().error('Not enough data to generate plots!')
#             return
            
#         # Calculate statistics
#         avg_velocity = self.calculate_average_velocity()
#         stable_velocity = self.calculate_stable_velocity()
#         total_displacement = self.positions[-1] - self.positions[0]
        
#         # Print summary
#         self.print_summary(avg_velocity, stable_velocity, total_displacement)
        
#         # Generate plots
#         timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
#         # 1. Combined plot (v-t and x-t)
#         self.create_combined_plot(timestamp)
        
#         # 2. Detailed analysis plot
#         self.create_analysis_plot(timestamp, avg_velocity, stable_velocity)
        
#         self.get_logger().info('\n✅ Plots saved successfully!')
        
#     def calculate_average_velocity(self):
#         """Calculate overall average velocity"""
#         if len(self.velocities) == 0:
#             return 0.0
#         return np.mean(self.velocities)
        
#     def calculate_stable_velocity(self):
#         """Calculate velocity in stable region"""
#         # Filter velocities in stable range (80-120% of target)
#         lower = self.target_velocity_mm * 0.8
#         upper = self.target_velocity_mm * 1.2
#         stable_vels = [v for v in self.velocities if lower < v < upper]
        
#         if len(stable_vels) > 0:
#             return np.mean(stable_vels)
#         return 0.0
        
#     def print_summary(self, avg_vel, stable_vel, displacement):
#         """Print test summary"""
#         self.get_logger().info('='*60)
#         self.get_logger().info('TEST SUMMARY')
#         self.get_logger().info('='*60)
#         self.get_logger().info(f'Target Velocity:        {self.target_velocity_mm:.2f} mm/s')
#         self.get_logger().info(f'Average Velocity:       {avg_vel:.2f} mm/s ({avg_vel/self.target_velocity_mm*100:.1f}%)')
#         self.get_logger().info(f'Stable Phase Velocity:  {stable_vel:.2f} mm/s ({stable_vel/self.target_velocity_mm*100:.1f}%)')
#         self.get_logger().info(f'Total Displacement:     {displacement:.2f} mm')
#         self.get_logger().info(f'Test Duration:          {self.timestamps[-1]:.2f} s')
#         self.get_logger().info(f'Data Points:            {len(self.positions)}')
#         self.get_logger().info('='*60 + '\n')
        
#     def create_combined_plot(self, timestamp):
#         """Create combined v-t and x-t plot"""
#         fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
#         # Velocity-Time plot
#         vel_times = self.timestamps[1:]  # Velocity has one less point
#         ax1.plot(vel_times, self.velocities, 'b-', linewidth=2, alpha=0.7, label='Actual Velocity')
#         ax1.axhline(y=self.target_velocity_mm, color='r', linestyle='--', linewidth=2, 
#                    label=f'Target Velocity ({self.target_velocity_mm:.1f} mm/s)')
        
#         # Mark stable region
#         lower = self.target_velocity_mm * 0.8
#         upper = self.target_velocity_mm * 1.2
#         stable_indices = [i for i, v in enumerate(self.velocities) if lower < v < upper]
        
#         if stable_indices:
#             stable_start = vel_times[stable_indices[0]]
#             stable_end = vel_times[stable_indices[-1]]
#             ax1.axvspan(stable_start, stable_end, alpha=0.2, color='green', label='Stable Phase')
            
#             # Calculate stable average
#             stable_vels = [self.velocities[i] for i in stable_indices]
#             avg_stable = np.mean(stable_vels)
#             mid_time = (stable_start + stable_end) / 2
#             ax1.text(mid_time, avg_stable + 2, 
#                     f'Avg: {avg_stable:.2f} mm/s\n({avg_stable/self.target_velocity_mm*100:.1f}%)',
#                     ha='center', fontsize=9, 
#                     bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        
#         ax1.set_xlabel('Time (s)', fontsize=11)
#         ax1.set_ylabel('Velocity (mm/s)', fontsize=11)
#         ax1.set_title('(a) Velocity-Time Curve', fontsize=12, fontweight='bold', loc='left')
#         ax1.grid(True, alpha=0.3)
#         ax1.legend(loc='best', fontsize=9)
        
#         # Position-Time plot
#         ax2.plot(self.timestamps, self.positions, 'g-', linewidth=2, label='Actual Position')
        
#         # Theoretical position (constant velocity)
#         theoretical_pos = [self.positions[0] + self.target_velocity_mm * t 
#                           for t in self.timestamps]
#         ax2.plot(self.timestamps, theoretical_pos, 'r--', linewidth=2, 
#                 label=f'Theoretical Position ({self.target_velocity_mm:.1f} mm/s)', alpha=0.6)
        
#         ax2.set_xlabel('Time (s)', fontsize=11)
#         ax2.set_ylabel('Position X (mm)', fontsize=11)
#         ax2.set_title('(b) Position-Time Curve', fontsize=12, fontweight='bold', loc='left')
#         ax2.grid(True, alpha=0.3)
#         ax2.legend(loc='best', fontsize=9)
        
#         plt.tight_layout()
        
#         filename = f'velocity_test_combined_{timestamp}.png'
#         plt.savefig(filename, dpi=300, bbox_inches='tight')
#         self.get_logger().info(f'📊 Saved: {filename}')
#         plt.close()
        
#     def create_analysis_plot(self, timestamp, avg_vel, stable_vel):
#         """Create detailed analysis plot with statistics"""
#         fig = plt.figure(figsize=(14, 8))
#         gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)
        
#         # Main plot: Velocity-Time with phase annotations
#         ax_main = fig.add_subplot(gs[0, :])
#         vel_times = self.timestamps[1:]
        
#         ax_main.plot(vel_times, self.velocities, 'b-', linewidth=2, alpha=0.7)
#         ax_main.axhline(y=self.target_velocity_mm, color='r', linestyle='--', linewidth=2)
        
#         # Annotate phases
#         n = len(self.velocities)
#         phases = [
#             (0, n//5, 'yellow', 'Startup'),
#             (n//5, 2*n//5, 'orange', 'Acceleration'),
#             (2*n//5, 3*n//5, 'green', 'Stable'),
#             (3*n//5, 4*n//5, 'orange', 'Deceleration'),
#             (4*n//5, n, 'gray', 'Stop')
#         ]
        
#         for start_idx, end_idx, color, label in phases:
#             if start_idx < len(vel_times) and end_idx <= len(vel_times):
#                 ax_main.axvspan(vel_times[start_idx], 
#                               vel_times[min(end_idx-1, len(vel_times)-1)], 
#                               alpha=0.15, color=color)
#                 mid_idx = (start_idx + end_idx) // 2
#                 if mid_idx < len(vel_times):
#                     ax_main.text(vel_times[mid_idx], max(self.velocities)*0.95, label,
#                                ha='center', fontsize=9, style='italic')
        
#         ax_main.set_xlabel('Time (s)', fontsize=11)
#         ax_main.set_ylabel('Velocity (mm/s)', fontsize=11)
#         ax_main.set_title('TM Robot PVT Velocity Control - Complete Motion Analysis', 
#                          fontsize=13, fontweight='bold')
#         ax_main.grid(True, alpha=0.3)
        
#         # Lower left: Velocity histogram
#         ax_hist = fig.add_subplot(gs[1, 0])
#         ax_hist.hist(self.velocities, bins=30, color='skyblue', edgecolor='black', alpha=0.7)
#         ax_hist.axvline(x=self.target_velocity_mm, color='r', linestyle='--', 
#                        linewidth=2, label='Target Velocity')
#         ax_hist.set_xlabel('Velocity (mm/s)', fontsize=10)
#         ax_hist.set_ylabel('Count', fontsize=10)
#         ax_hist.set_title('Velocity Distribution', fontsize=11, fontweight='bold')
#         ax_hist.legend(fontsize=8)
#         ax_hist.grid(True, alpha=0.3)
        
#         # Lower right: Statistics table
#         ax_table = fig.add_subplot(gs[1, 1])
#         ax_table.axis('off')
        
#         max_vel = np.max(self.velocities)
#         min_vel = np.min(self.velocities)
        
#         lower = self.target_velocity_mm * 0.8
#         upper = self.target_velocity_mm * 1.2
#         stable_count = len([v for v in self.velocities if lower < v < upper])
#         stable_ratio = stable_count / len(self.velocities) * 100
        
#         displacement = self.positions[-1] - self.positions[0]
        
#         table_data = [
#             ['Parameter', 'Value', 'Achievement'],
#             ['Target Velocity', f'{self.target_velocity_mm:.2f} mm/s', '100%'],
#             ['Average Velocity', f'{avg_vel:.2f} mm/s', f'{avg_vel/self.target_velocity_mm*100:.1f}%'],
#             ['Stable Phase Avg', f'{stable_vel:.2f} mm/s', f'{stable_vel/self.target_velocity_mm*100:.1f}%'],
#             ['Max Velocity', f'{max_vel:.2f} mm/s', f'{max_vel/self.target_velocity_mm*100:.1f}%'],
#             ['Min Velocity', f'{min_vel:.2f} mm/s', '-'],
#             ['Stable Phase Ratio', f'{stable_ratio:.1f}%', '-'],
#             ['Total Displacement', f'{displacement:.2f} mm', '-'],
#             ['Test Duration', f'{self.timestamps[-1]:.2f} s', '-'],
#         ]
        
#         table = ax_table.table(cellText=table_data, cellLoc='center', loc='center',
#                               colWidths=[0.4, 0.3, 0.3])
#         table.auto_set_font_size(False)
#         table.set_fontsize(9)
#         table.scale(1, 2)
        
#         # Header formatting
#         for i in range(3):
#             table[(0, i)].set_facecolor('#4CAF50')
#             table[(0, i)].set_text_props(weight='bold', color='white')
        
#         # Highlight stable phase row
#         table[(3, 0)].set_facecolor('#E8F5E9')
#         table[(3, 1)].set_facecolor('#E8F5E9')
#         table[(3, 2)].set_facecolor('#E8F5E9')
        
#         ax_table.set_title('Performance Statistics', fontsize=11, fontweight='bold', pad=20)
        
#         filename = f'velocity_test_analysis_{timestamp}.png'
#         plt.savefig(filename, dpi=300, bbox_inches='tight')
#         self.get_logger().info(f'📊 Saved: {filename}')
#         plt.close()


# def main():
#     """Main function"""
#     rclpy.init()
    
#     # Test parameters (you can modify these)
#     TEST_DURATION = 3.0  # seconds
#     TARGET_VELOCITY = 0.01  # m/s (10 mm/s)
    
#     print('\n' + '='*60)
#     print('TM Robot PVT Velocity Control Test')
#     print('='*60)
#     print(f'Target Velocity: {TARGET_VELOCITY*1000:.1f} mm/s')
#     print(f'Test Duration: {TEST_DURATION:.1f} seconds')
#     print('='*60 + '\n')
    
#     # Create and run test node
#     node = VelocityTestNode(
#         test_duration=TEST_DURATION,
#         target_velocity=TARGET_VELOCITY
#     )
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n\n⚠️  Test interrupted by user')
#         node.get_logger().info('Generating plots from collected data...')
#         if len(node.positions) > 1:
#             node.generate_plots()
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()