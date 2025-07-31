#!/usr/bin/env python3
from time import sleep
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy

class TmRobotMove(Node):
    def __init__(self):
        super().__init__('tm_robot_move_node')
        self.moveit_py = MoveItPy(node_name="tm_moveitpy_demo")
        self.tmroobot = self.moveit_py.get_planning_component("tmr_arm")
        
        self.tmroobot.set_planning_pipeline_id("ompl")
        
        planning_options = self.tmroobot.get_planning_options()
        planning_options.planning_time = 5.0
        planning_options.planning_attempts = 10
        
        self.tmroobot.set_goal_position_tolerance(0.01)
        self.tmroobot.set_goal_orientation_tolerance(0.01)
        self.tmroobot.set_goal_tolerance(0.01)
        self.tmroobot.set_max_velocity_scaling_factor(1.0)
        self.tmroobot.set_max_acceleration_scaling_factor(1.0)
        
    def run_demo(self):
        while True:
            self.tmroobot.set_random_joint_goal()
            result = self.tmroobot.plan_and_execute()
            print(f"Execution result: {result}")
            sleep(0.5)
            

def main():
    rclpy.init()
    node = TmRobotMove()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3

# import rclpy
# from moveit.planning import MoveItPy, PlanningComponent
# from geometry_msgs.msg import Pose
# import time

# def main():
#     # Initialize ROS2
#     rclpy.init()
    
#     try:
#         # Wait a moment for all services to be available
#         time.sleep(2)
        
#         # Initialize MoveItPy - this will connect to existing move_group node
#         print("Connecting to MoveIt...")
#         moveit = MoveItPy(node_name="moveit_demo")
        
#         print("Getting robot...")
#         moveit_robot = moveit.get_robot()
        
#         print("Creating planning component...")
#         # Use the correct planning group name for your robot
#         planning_component = PlanningComponent("tmr_arm", moveit_robot)
        
#         # Set target pose
#         print("Setting target pose...")
#         target_pose = Pose()
#         target_pose.position.x = 0.4
#         target_pose.position.y = 0.0
#         target_pose.position.z = 0.2
#         target_pose.orientation.w = 1.0  # No rotation
        
#         # Set planning goal - try different end effector names
#         print("Setting planning goal...")
#         try:
#             # Try common end effector link names
#             end_effector_links = ["tool0", "end_effector", "flange", "tm_tool0", "link_6"]
            
#             for link_name in end_effector_links:
#                 try:
#                     planning_component.set_goal_state(
#                         pose=target_pose, 
#                         pose_reference_frame="base_link",
#                         pose_link=link_name
#                     )
#                     print(f"Successfully set goal with end effector link: {link_name}")
#                     break
#                 except Exception as e:
#                     print(f"Failed with link {link_name}: {e}")
#                     continue
#             else:
#                 print("Failed to set goal with any end effector link, trying without pose_link...")
#                 planning_component.set_goal_state(pose=target_pose, pose_reference_frame="base_link")
        
#         except Exception as e:
#             print(f"Error setting goal: {e}")
#             return
        
#         # Plan motion
#         print("Planning motion...")
#         plan_result = planning_component.plan()
        
#         if plan_result:
#             print("Planning successful!")
#             robot_trajectory = plan_result.trajectory
            
#             # Try to execute - check what controllers are available
#             print("Attempting to execute...")
#             try:
#                 # Try different controller names
#                 controller_names = ["tmr_arm_controller", "arm_controller", "manipulator_controller"]
                
#                 for controller in controller_names:
#                     try:
#                         print(f"Trying controller: {controller}")
#                         success = moveit_robot.execute(robot_trajectory, controllers=[controller])
#                         if success:
#                             print(f"Execution successful with controller: {controller}")
#                             break
#                     except Exception as e:
#                         print(f"Failed with controller {controller}: {e}")
#                         continue
#                 else:
#                     # Try without specifying controllers
#                     print("Trying execution without specifying controllers...")
#                     success = moveit_robot.execute(robot_trajectory)
#                     if success:
#                         print("Execution successful!")
#                     else:
#                         print("Execution failed!")
                        
#             except Exception as e:
#                 print(f"Execution error: {e}")
#         else:
#             print("Planning failed!")
            
#     except Exception as e:
#         print(f"Error: {e}")
#         import traceback
#         traceback.print_exc()
#     finally:
#         # Shutdown
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


