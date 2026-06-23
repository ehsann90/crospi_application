#!/usr/bin/env python3

import sys
import rclpy

from betfsm import (
    SUCCEED, TICKING, ABORT, CANCEL, TIMEOUT,
    get_logger, set_logger,
    TickingStateMachine, 
    TimedWait
)

# TickingState is not exported by betfsm.__init__ in this repo version.
from betfsm.betfsm import TickingState

from betfsm_ros import BeTFSMNode, ROSRunner
from betfsm_ros.events_ros import TopicEventReceiver

from betfsm_crospi import load_task_list, CrospiTask, CrospiDeactivate


class WaitForSkillEvent(TickingState):
    """
    Blocks the FSM until a fresh std_msgs/String event is received.

    Topic:
        /ala/skill_event

    Expected message:
        std_msgs/String(data="start_reaching")
    """

    def __init__(
        self,
        name,
        topic_name="/ala/skill_event",
        event_name="start_reaching",
        max_age=2.0,
        queue_size=10
    ):
        super().__init__(name, [SUCCEED, TICKING, ABORT])
        self.topic_name = topic_name
        self.event_name = event_name
        self.max_age = max_age
        self.queue_size = queue_size

        node = BeTFSMNode.get_instance()
        self.receiver = TopicEventReceiver.get_instance(
            node,
            self.topic_name,
            self.queue_size
        )

        # Clear any stale start_reaching event from previous tests.
        self.receiver.clear()

        self.has_logged_waiting = False

    def doo(self, blackboard):
        if not self.has_logged_waiting:
            get_logger().info(
                f"Waiting for std_msgs/String(data='{self.event_name}') on {self.topic_name}"
            )
            self.has_logged_waiting = True

        matched = self.receiver.poll_recent_for(
            [self.event_name],
            self.max_age
        )

        if matched == self.event_name:
            get_logger().info(f"Received skill event: {matched}")
            self.has_logged_waiting = False
            return SUCCEED

        return TICKING


class AlaReachingSkill(TickingStateMachine):
    """
    Proper episode loop:

        IdleHoldStart:
            loads and activates idle.etasl.lua, then leaves Crospi running

        WaitForStart:
            blocks until /ala/skill_event = start_reaching

        StopIdle:
            deactivates/cleans idle task

        ReachAssist:
            runs reaching task until its exit monitor fires

        Then return to IdleHoldStart.
    """

    def __init__(self):
        super().__init__("ala_reaching_skill_sm", [SUCCEED, ABORT])

        self.add_state(
            TimedWait("StartupWait", 2.0),
            transitions={
                SUCCEED: "IdleHoldStart",
                ABORT: ABORT,
                CANCEL: ABORT,
                TIMEOUT: ABORT
            }
        )

        self.add_state(
            CrospiTask(
                "IdleHoldStart",
                "IdleHold",
                node=None,
                event_check=False
            ),
            transitions={
                SUCCEED: "WaitForStart",
                ABORT: ABORT,
                CANCEL: ABORT,
                TIMEOUT: ABORT
            }
        )

        self.add_state(
            WaitForSkillEvent("WaitForStart"),
            transitions={
                SUCCEED: "reset",
                ABORT: ABORT,
                CANCEL: ABORT,
                TIMEOUT: ABORT
            }
        )

        self.add_state(
            CrospiDeactivate(force_outcome=SUCCEED),
            transitions={
                SUCCEED: "ReachAssist",
                ABORT: ABORT,
                CANCEL: ABORT,
                TIMEOUT: ABORT
            }
        )

        self.add_state(
            CrospiTask("ReachAssist", "ReachAssist", node=None),
            transitions={
                SUCCEED: "IdleHoldStart",
                ABORT: ABORT,
                CANCEL: ABORT,
                TIMEOUT: ABORT
            }
        )


def main(args=None):
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("ala_reaching_skill")

    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    set_logger("state", my_node.get_logger())

    get_logger().info("ALA reaching skill started")
    get_logger().info(
        "Publish std_msgs/String 'start_reaching' on /ala/skill_event to start one reaching episode."
    )

    blackboard = {}

    load_task_list(
        "$[crospi_application]/skill_specifications/libraries/ala_reaching_skill/tasks/ala_reaching_skill.json",
        blackboard
    )

    sm = AlaReachingSkill()

    runner = ROSRunner(
        my_node,
        sm,
        blackboard,
        frequency=100.0,
        publish_frequency=5.0,
        debug=False,
        display_active=False
    )

    try:
        runner.run()
    except KeyboardInterrupt:
        get_logger().info("KeyboardInterrupt received, deactivating Crospi.")
        try:
            cleanup_sm = CrospiDeactivate(force_outcome=ABORT)
            cleanup_runner = ROSRunner(
                my_node,
                cleanup_sm,
                blackboard,
                frequency=100.0,
                publish_frequency=5.0,
                debug=False,
                display_active=False
            )
            cleanup_runner.run()
        except Exception:
            pass

        my_node.destroy_node()
        return

    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
