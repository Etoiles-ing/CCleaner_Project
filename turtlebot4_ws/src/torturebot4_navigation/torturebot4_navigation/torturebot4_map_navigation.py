import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    navigator.info('Initialising pose')
    
    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.info('Waiting nav2')
    navigator.waitUntilNav2Active()

    # Set goal poses
    navigator.info('Setting goal poses')
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.0, 0.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([0.0, 5.0], TurtleBot4Directions.EAST))
    # goal_pose.append(navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.SOUTH))
    # goal_pose.append(navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.NORTH))

    # Undock
    navigator.info('Undocking')
    navigator.undock()

    # Follow Waypoints
    navigator.info('Starting')
    navigator.startFollowWaypoints(goal_pose)

    # Finished navigating, dock
    navigator.info('Docking')
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
