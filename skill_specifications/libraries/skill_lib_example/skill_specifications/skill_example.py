#!/usr/bin/env python3

#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

import rclpy
import sys

from betfsm import (
    SUCCEED, TICKING, CANCEL, TIMEOUT,ABORT,NO_EVENT,
    get_logger,set_logger,Sequence, Repeat, Message, TimedWait,
    EventSequential, EventConcurrent, ConcurrentSequence, TickingStateMachine
)

from betfsm_ros import BeTFSMNode, ROSRunner
                        
from betfsm_crospi import load_task_list, CrospiTask


import math


class MyStateMachine(TickingStateMachine):
    def __init__(self):
        super().__init__("my_state_machine",[SUCCEED, ABORT])

        self.add_state(
            CrospiTask("MovingHome","MovingHome",node=None), 
            transitions={SUCCEED: "MovingDown", 
                        ABORT: ABORT}
        )

        self.add_state(
            CrospiTask("MovingDown","MovingDown",node=None), 
            transitions={SUCCEED: "MovingUp", 
                        ABORT: ABORT}
        )

        self.add_state(
            CrospiTask("MovingUp","MovingUp",node=None), 
            transitions={SUCCEED: "MovingSpline", 
                        ABORT: ABORT}
        )

        self.add_state(
            CrospiTask("MovingSpline","MovingSpline",node=None), 
            transitions={SUCCEED: "pause_state", 
                        ABORT: ABORT}
        )

        self.add_state(
            TimedWait("pause_state", 5.0),# wait for 5 seconds             
            transitions={SUCCEED: "MovingHome", 
                        ABORT: ABORT})


# main
def main(args=None):

    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    #set_logger("service",my_node.get_logger()) 
    set_logger("state",my_node.get_logger())

    get_logger().info("skill_example started")

    blackboard = {}

    load_task_list("$[crospi_application]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    
    sm = MyStateMachine()

    runner = ROSRunner(
        my_node,sm,blackboard, 
        frequency=100.0, 
        publish_frequency=5.0, 
        debug=False, 
        display_active=False)

    try:
        runner.run()
    except KeyboardInterrupt:
        my_node.destroy_node()
        return   
    my_node.destroy_node()
    rclpy.shutdown()
    print("shutdown")
        

if __name__ == "__main__":
    sys.exit(main(sys.argv))
