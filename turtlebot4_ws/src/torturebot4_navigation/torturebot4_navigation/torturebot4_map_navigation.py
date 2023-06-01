import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    print("Init")
    rclpy.init()

    print("Load Navigator")
    navigator = TurtleBot4Navigator()

    print("Navigator Loaded")
    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    navigator.info('Initialising pose')
    
    # Set initial pose
    initial_pose = navigator.getPoseStamped([-0.37531, 0.32600], TurtleBot4Directions.NORTH)
    print("Initial Pose set at : ")
    print(initial_pose)
    
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.info('Waiting nav2')
    navigator.waitUntilNav2Active()

    # Set goal poses
    navigator.info('Setting goal poses')
    goal_pose = []
    # goal_pose = navigator.getPoseStamped([-1.5336, -5.1547], TurtleBot4Directions.EAST)

    goal_pose.append(navigator.getPoseStamped([-1.57920, 0.46241], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-1.62486, -1.47442], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-4.73604, -1.54578], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-4.98320, -5.02000], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([-1.00999, -5.02574], TurtleBot4Directions.NORTH))
    

    # Undock
    navigator.info('Undocking')
    navigator.undock()

    # Follow Waypoints
    navigator.info('Starting')
    navigator.startFollowWaypoints(goal_pose)
    # navigator.startToPose(goal_pose)

    # Finished navigating, dock
    navigator.info('Docking')
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
