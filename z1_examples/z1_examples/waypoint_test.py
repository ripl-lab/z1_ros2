#!/usr/bin/env python3

# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory



class TrajectoryCaller(Node):

    def __init__(self):
        super().__init__("joint_trajectory_caller")

        self.get_logger().info("Creating client")
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.action_client.wait_for_server()
        self.get_logger().info("Connection established")

    def send_traj_request(self, positions: list[np.ndarray]):
        self.get_logger().info("Creating trajectory request")
        traj_msg = FollowJointTrajectory.Goal()
        traj: JointTrajectory = traj_msg.trajectory
        traj.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        for i, pos in enumerate(positions):
            self.get_logger().info(f"{pos}")
            pt = JointTrajectoryPoint()
            pt.positions = pos.tolist()
            pt.time_from_start.sec = (i+1) * 2
            traj.points.append(pt)

        self.get_logger().info("Sending trajectory request")
        return self.action_client.send_goal_async(traj_msg)



def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryCaller()

    future = node.send_traj_request([
        np.array([0.0, 2.8, -1.2, -1.0, 0.0, 0.0]),
        np.array([-0.15, 2.8, -1.0, -1.3, 0.0, 0.0]),
        np.array([-0.15, 2.8, -1.0, -1.3, 0.0, 0.0]),
        np.array([-0.15, 2.7, -1.0, -1.3, 0.0, 0.0]),
        np.array([-0.15, 2.6, -1.0, -1.3, 0.0, 0.0]),
        np.array([-0.15, 2.5, -1.0, -1.3, 0.0, 0.0]),
        np.array([-0.15, 2.4, -1.0, -1.3, 0.0, 0.0]),
        np.array([0.1, 2.3, -0.9, -1.5, 0.0, 0.0]),
        np.array([0.2, 2.1, -0.7, -1.5, 0.0, 0.0]),
        np.array([0.2, 2.0, -0.6, -1.5, 0.0, 0.0]),
        np.array([-0.2, 2.5, -1.2, -1.1, 0.6, 0.0]),
        np.array([-0.2, 2.5, -1.2, -1.0, 0.7, 0.0]),
        np.array([-0.2, 2.5, -1.2, -0.9, 0.8, 0.0]),

        # #Come back from tiny branch
        # np.array([-0.3, 2.3, -1.3, -1.0, 0.4, 0.0]),
        # np.array([-0.3, 2.3, -1.1, -1.0, 0.4, 0.0]),

        # #"Get back and see tree"),s
        # np.array([-0.3, 2.3, -1.1, -1.1, 0.4, 0.0]),
        # np.array([-0.3, 2.3, -0.9, -1.2, 0.4, 0.0]),
        # np.array([-0.3, 1.9, -0.6, -1.2, 0.4, 0.0]),


        # # #see right side
        # np.array([-0.4, 1.9, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.5, 1.9, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.5, 1.8, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.6, 1.8, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.6, 1.7, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.7, 1.7, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.8, 1.7, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.7, 1.6, -0.6, -1.2, 0.4, 0.0]),
        
        # np.array([-0.6, 1.6, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.8, 1.6, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.9, 1.6, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.7, 1.6, -0.6, -1.2, 0.4, 0.0]),

        # np.array([-0.6, 1.5, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.7, 1.5, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.8, 1.5, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.6, 1.4, -0.6, -1.2, 0.4, 0.0]),

        # # #most right branch going up
        # np.array([-0.5, 1.4, -0.6, -1.2, 0.4, 0.0]),
        # np.array([-0.5, 1.2, -0.6, -1.0, 0.4, 0.0]),
        # np.array([-0.6, 1.2, -0.6, -1.0, 0.4, 0.0]),
        # np.array([-0.7, 1.2, -0.6, -1.0, 0.4, 0.0]),
        # np.array([-0.8, 1.2, -0.6, -1.0, 0.4, 0.0]),
        # np.array([-0.9, 1.2, -0.6, -1.0, 0.4, 0.0]),

        # np.array([-0.9, 1.1, -0.6, -0.9, 0.4, 0.0]),
        # np.array([-0.9, 1.1, -0.6, -0.8, 0.4, 0.0]),
        # np.array([-0.9, 1.2, -0.7, -0.8, 0.4, 0.0]),
        # np.array([-0.7, 1.7, -1.0, -0.6, 0.5, 0.0]),
        # np.array([-0.9, 1.7, -1.0, -0.7, 0.6, 0.0]),
        # np.array([-0.8, 1.7, -1.1, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.7, -1.2, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.7, -1.3, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.7, -1.4, -0.6, 0.6, 0.0]),

        # np.array([-0.9, 1.7, -1.4, -0.6, 0.6, 0.0]),
        # np.array([-0.9, 1.7, -1.5, -0.6, 0.6, 0.0]),
        # np.array([-0.9, 1.7, -1.6, -0.6, 0.6, 0.0]),
        # np.array([-1.0, 1.7, -1.6, -0.6, 0.6, 0.0]),
        # np.array([-1.0, 1.7, -1.7, -0.6, 0.6, 0.0]),
        
        # # #most right branch coming down
        # np.array([-0.9, 1.7, -1.7, -0.6, 0.6, 0.0]),
        # np.array([-0.9, 1.7, -1.6, -0.6, 0.6, 0.0]),
        # np.array([-0.9, 1.7, -1.5, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.7, -1.5, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.7, -1.4, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.6, -1.2, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.6, -1.1, -0.6, 0.6, 0.0]),
        # np.array([-0.8, 1.6, -1.0, -0.6, 0.6, 0.0]),
        # np.array([-0.7, 1.6, -1.0, -0.6, 0.6, 0.0]),
        # np.array([-0.7, 1.6, -1.0, -0.5, 0.6, 0.0]),
        # np.array([-0.6, 1.6, -1.0, -0.6, 0.6, 0.0]),


        # #Going up the back branches
        # np.array([-0.6, 1.5,-1., -0.5, 0.4, 0.0]),
        # np.array([-0.6, 1.4, -1., -0.5, 0.4, 0.0]),
        # np.array([-0.7, 1.3, -1., -0.5, 0.4, 0.0]),
        # np.array([-0.7, 1.2, -1.1,-0.5, 0.4, 0.0]),
        # np.array([-0.8, 1.2, -1.1,-0.5, 0.4, 0.0]),
        # np.array([-0.8, 1.2, -1.2,-0.5, 0.4, 0.0]),
        # np.array([-0.8, 1.2, -1.3,-0.5, 0.4, 0.0]),
        # np.array([-0.8, 1.2, -1.3,-0.6, 0.4, 0.0]),
        # np.array([-0.7, 1.2, -1.3,-0.7, 0.4, 0.0]),
        # np.array([-0.6,1.2, -1.3,-0.7, 0.4, 0.0]),
        # np.array([-0.6,1.0, -1.3, -0.1,0.4, 0.0]),
        # np.array([-0.6, 1.0, -1.3, -0.2,0.4, 0.0]),
        # np.array([-0.6, 1.0, -1.2, -0.2,0.4, 0.0]),
        # np.array([-0.6, 1.0, -1.1, -0.2,0.4, 0.0]),
        # np.array([-0.6, 1.0, -1.0, -0.2,0.4, 0.0]),
        # np.array([-0.6, 1.0, -0.9, -0.2,0.4, 0.0]),

        # #goinf back to the other side
        # np.array([-0.8,1.0, -0.9, -0.2,0.4, 0.0]),
        # np.array([-0.9, 1.0, -0.9, -0.2,0.4, 0.0]),
        # np.array([-0.9, 1.9, -0.9, -1.2, 1.2, 0.0]),
        # np.array([-0.9, 1.9, -0.9, -1.0, 1.2, 0.0]),
        # np.array([-0.9, 1.9, -0.9, -0.8, 1.2, 0.0]),
        # np.array([-0.9, 1.9, -0.9, -0.7, 1.2, 0.0]),
        # np.array([-0.9, 1.9, -0.9, -0.4, 1.2, 0.0]),
        # np.array([-0.9, 1.9, -0.7, -0.4, 1.2, 0.0]),

        # #Transinp.array([-0.9, 1.9, -0.7, -0.4, 1.2tion code
        # np.array([-0.8, 1.9, -0.7, -0.4, 1.0, 0.0]),
        # np.array([-0.8, 1.9, -0.7, -0.4, 1.0, 0.0]),
        # np.array([-0.8, 1.9, -0.7, -0.4, 1.0, 0.0]),
        # np.array([-0.8, 1.9, -0.7, -0.4, 1.0, 0.0]),
        # np.array([-0.8, 1.9, -0.7, -0.4, 1.0, 0.0]),
        # np.array([-0.8, 1.9, -0.7, -0.4, 1.0, 0.0]),


       
    
        # # old plus new code
        # np.array([-0.1, 2.0, -1.0, -0.7, 0.0, 0.0]),
        # np.array([-0.1, 2.0, -1.0, -0.8, 0.0, 0.0]),
        # np.array([-0.1, 2.0, -1.0, -0.9, 0.0, 0.0]),

        # np.array([0.0, 2.0, -1.0, -0.9, 0.0, 0.0]),
        # np.array([0.0, 2.0, -1.1, -0.8, 0.0, 0.0]),
        # np.array([0.0, 2.0, -1.1, -0.9, 0.0, 0.0]),
        # np.array([0.0, 2.0, -1.1, -0.9, 0.1, 0.0]),

        # np.array([0.0, 2.0, -2.0, 0.4, 0.2, 0.0]),
        # np.array([0.0, 2.0, -2.0, 0.4, 0.3, 0.0]),
        # np.array([0.0, 2.0, -2.0, 0.4, 0.4, 0.0]),
        # np.array([0.0, 2.0, -2.0, 0.4, 0.5, 0.0]),
        # np.array([0.0, 2.0, -2.0, 0.4, 0.6, 0.0]),
        # np.array([0.0, 2.0, -2.0, 0.4, 0.7, 0.0]),
        # np.array([0.0, 2.0, -2.0, 0.4, 0.8, 0.0]),

        np.array([0.0, 2.0, -2.0, 0.7, 0.0, 0.0]),
        np.array([0.0, 2.0, -2.0, 0.7, 0.0, 0.0]),
        np.array([0.0, 1.9, -2.0, 0.6, 0.0, 0.0]),
        np.array([0.0, 1.8, -2.0, 0.6, 0.0, 0.0]),
        np.array([0.0, 1.8, -1.8, 0.6, 0.0, 0.0]),
        np.array([0.0, 1.7, -1.5, 0.5, 0.0, 0.0]),
        np.array([0.2, 1.7, -1.5, 0.4, 0.0, 0.0]),
        np.array([0.3, 1.7, -1.5, 0.4, 0.0, 0.0]),
        np.array([0.3, 1.7, -1.5, 0.3, 0.0, 0.0]),
        np.array([0.3, 1.7, -1.5, 0.2, 0.0, 0.0]),
        np.array([0.3, 1.7, -1.5, 0.1, 0.0, 0.0]),
        np.array([0.3, 1.7, -1.5, 0.0, 0.0, 0.0]),
        np.array([0.4, 1.7, -1.5, 0.1, 0.0, 0.0]),
        np.array([0.5, 1.7, -1.5, 0.1, 0.0, 0.0]),
        np.array([0.5, 1.7, -1.6, 0.1, 0.0, 0.0]),
        np.array([0.6, 1.7, -1.6, 0.1, 0.1, 0.0]),
        np.array([0.7, 1.7, -1.6, 0.1, 0.3, 0.0]),
       



        # np.array([0.4, 1.8, -1.0, -0.9, 0.0, 0.0]),
        # np.array([0.4, 1.6, -0.9, -0.9, 0.0, 0.0]),
        # np.array([0.5, 1.6, -0.9, -0.9, 0.0, 0.0]),
        # np.array([0.6, 1.6, -1.0, -0.9, 0.0, 0.0]),
        # np.array([0.7, 1.6, -1.0, -1.0, 0.0, 0.0]),
        # np.array([0.5, 1.6, -1.0, -0.9, 0.0, 0.0]),
        # np.array([0.4, 1.6, -1.0, -0.8, 0.0, 0.0]),
        # np.array([0.3, 1.6, -1.0, -0.7, 0.0, 0.0]),
        # np.array([0.3, 1.6, -1.0, -0.6, 0.0, 0.0]),
        # np.array([0.2, 1.6, -1.0, -0.6, 0.0, 0.0]),
        # np.array([0.2, 1.6, -1.0, -0.5, 0.0, 0.0]),
        # np.array([0.1, 1.6, -1.0, -0.5, 0.0, 0.0]),
        # np.array([0.1, 1.6, -1.0, -0.4, 0.0, 0.0]),
        # np.array([0.1, 1.6, -1.0, -0.3, 0.0, 0.0]),
        # np.array([0.0, 1.6, -1.0, -0.3, 0.0, 0.0]),
        # np.array([0.0, 1.6, -1.0, -0.2, 0.0, 0.0]),




        # np.array([0.0, 2.0, -2.0, 0.4, 0.0, 0.0]),
        # np.array([0.0, 1.8, -2.0, 0.4, 0.0, 0.0]),
        # np.array([0.0, 1.8, -1.8, 0.4, 0.0, 0.0]),
        # np.array([0.0, 1.8, -1.6, 0.4, 0.0, 0.0]),
        # np.array([0.3, 1.8, -1.6, 0.4, 0.0, 0.0]),
        # np.array([0.3, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.4, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.4, 1.8, -1.7, 0.0, 0.0, 0.0]),
        # np.array([0.4, 1.8, -1.8, 0.0, 0.1, 0.0]),
        # np.array([0.4, 1.8, -1.8, 0.0, 0.3, 0.0]),
        # np.array([0.4, 1.8, -1.8, 0.0, 0.4, 0.0]),
        # np.array([0.4, 1.8, -1.0, -0.9, 0.0, 0.0]),
        # np.array([0.4, 1.6, -0.9, -0.9, 0.0, 0.0]),
        # np.array([0.5, 1.6, -0.9, -0.9, 0.0, 0.0]),
        # np.array([0.6, 1.6, -1, -0.9, 0.0, 0.0]),
        # np.array([0.7, 1.6, -1, -1, 0.0, 0.0]),
        # np.array([0.5, 1.6, -1, -0.9, 0.0, 0.0]),
        # np.array([0.4, 1.6, -1, -0.8, 0.0, 0.0]),
        # np.array([0.3, 1.6, -1, -0.7, 0.0, 0.0]),
        # np.array([0.3, 1.6, -1, -0.6, 0.0, 0.0]),
        # np.array([0.2, 1.6, -1, -0.6, 0.0, 0.0]),
        # np.array([0.2, 1.6, -1, -0.5, 0.0, 0.0]),
        # np.array([0.1, 1.6, -1, -0.5, 0.0, 0.0]),
        # np.array([0.1, 1.6, -1, -0.4, 0.0, 0.0]),
        # np.array([0.1, 1.6, -1, -0.3, 0.0, 0.0]),
        # np.array([0.0, 1.6, -1, -0.3, 0.0, 0.0]),
        # np.array([0.0, 1.6, -1, -0.2, 0.0, 0.0]),

        # #return home and get the last branch
        # np.array([0.0, 1.6, -1.0, -0.2, -0.3, 0.0]),
        # np.array([0.0, 1.4, -0.8, -0.2, -0.3, 0.0]),
        # np.array([0.0, 1.2, -0.6, -0.2, -0.3, 0.0]),
        # np.array([0.0, 0.9, -0.4, -0.2, -0.3, 0.0]),
        # np.array([0.0, 0.7, -0.4, -0.2, -0.3, 0.0]),
        # np.array([0.0, 0.7, -0.3, -0.2, -0.3, 0.0]),
        # np.array([0.0, 0.7, -0.2, -0.2, -0.3, 0.0]),
        



        # np.array([0.5, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.5, 1.8, -1.6, -0.1, 0.0, 0.0]),
        # np.array([0.6, 1.8, -1.6, -0.2, 0.0, 0.0]),
        # np.array([0.7, 1.8, -1.6, -0.2, 0.0, 0.0]),
        # np.array([0.8, 1.8, -1.6, -0.3, 0.0, 0.0]),

        # np.array([0.7, 1.8, -1.6, -0.2, 0.0, 0.0]),
        # np.array([0.6, 1.8, -1.6, -0.2, 0.0, 0.0]),
        # np.array([0.5, 1.8, -1.6, -0.1, 0.0, 0.0]),
        # np.array([0.5, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.4, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.3, 1.8, -1.6, 0.1, 0.0, 0.0]),
        # np.array([0.3, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.1, 1.8, -1.6, 0.0, 0.0, 0.0]),
        # np.array([0.1, 1.8, -1.6, 0.2, 0.0, 0.0]),

        ##np.array([-0.3, 0.9, -0.6, -0.4, 0.1, 0.0]),
         # #"see tree"),
        # np.array([0.0, 1.5, -1, -0.4, 0.0, 0.0]),
        
    ])
    rclpy.spin_until_future_complete(node, future)



if __name__ == '__main__':
    main()
