#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import os
import time

class GazeboSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_spawner')

        # Create clients for spawning and deleting objects
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        # Requests
        self.spawn_req = SpawnEntity.Request()
        self.delete_req = DeleteEntity.Request()

        # Wait for Gazebo services
        self.get_logger().info("Waiting for Gazebo services...")
        while not self.spawn_cli.wait_for_service(timeout_sec=2.0) or not self.delete_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /spawn_entity and /delete_entity services...")

    def clear_scene(self):
        """Deletes all known models in the Gazebo world."""
        known_models = ["drc_practice_orange_jersey_barrier", "drc_practice_white_jersey_barrier", "construction_cone"]
        
        for model in known_models:
            self.delete_req.name = model
            future = self.delete_cli.call_async(self.delete_req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Deleted: {model}")

    def spawn_object(self, model_name, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """Spawns a Gazebo object from the model directory."""
        # Try to get the model from GAZEBO_MODEL_PATH
        gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH', os.path.expanduser("~/.gazebo/models"))
        model_path = f"{gazebo_model_path}/{model_name}/model.sdf"

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model '{model_name}' not found! Make sure it exists in {gazebo_model_path}")
            return False

        # Read SDF file
        with open(model_path, 'r') as file:
            self.spawn_req.xml = file.read()

        self.spawn_req.name = model_name
        self.spawn_req.robot_namespace = model_name
        self.spawn_req.initial_pose.position.x = x
        self.spawn_req.initial_pose.position.y = y
        self.spawn_req.initial_pose.position.z = z

        from transforms3d.euler import euler2quat
        q = euler2quat(roll, pitch + 3.14159, yaw)  # Add 180 degrees (π radians) to roll
        self.spawn_req.initial_pose.orientation.x = q[0]
        self.spawn_req.initial_pose.orientation.y = q[1]
        self.spawn_req.initial_pose.orientation.z = q[2]
        self.spawn_req.initial_pose.orientation.w = q[3]

        future = self.spawn_cli.call_async(self.spawn_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    spawner = GazeboSpawner()

    # Clear the scene before spawning new objects
    spawner.clear_scene()

    # Spawn new objects
    # spawner.spawn_object("drc_practice_orange_jersey_barrier", x=2.0, y=-4.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
    # spawner.spawn_object("drc_practice_white_jersey_barrier", x=-3.0, y=-1.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
    # spawner.spawn_object("drc_practice_orange_jersey_barrier", x=-1.0, y=-4.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
    # spawner.spawn_object("drc_practice_white_jersey_barrier", x=-2.0, y=-3.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)

    spawner.spawn_object(
        "drc_practice_white_jersey_barrier",
        x=1.0,
        y=-1.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0
    )

    spawner.spawn_object(
        "drc_practice_orange_jersey_barrier",
        x=2.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0
    )









    # spawner.spawn_object("construction_cone", x=0.5, y=0.5, z=1.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
