# Software License Agreement (BSD License)
#
# Copyright (c) 2023, PickNik Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import annotations

import time
from threading import Thread
from typing import TYPE_CHECKING, Any, Generic, cast

from rclpy.action import ActionClient
from rclpy.expand_topic_name import expand_topic_name

from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_action_class,
    get_action_goal_instance,
)
from rosbridge_library.internal.type_support import (
    ROSActionFeedbackT,
    ROSActionGoalT,
    ROSActionResultT,
)

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.action.client import ClientGoalHandle
    from rclpy.node import Node
    from rclpy.task import Future

    from rosbridge_library.internal.type_support import (
        FeedbackMessage,
        ROSMessage,
    )


class InvalidActionException(Exception):
    def __init__(self, action_name: str) -> None:
        Exception.__init__(self, f"Action {action_name} does not exist")


def args_to_action_goal_instance(inst: ROSMessage, args: list | dict[str, Any] | None) -> None:
    """
    Populate an action goal instance with the provided args.

    Propagates any exceptions that may be raised.

    :param args: Can be a dictionary of values, or a list, or None
    """
    msg = {}
    if isinstance(args, list):
        msg = dict(zip(inst.get_fields_and_field_types().keys(), args, strict=False))
    elif isinstance(args, dict):
        msg = args

    # Populate the provided instance, propagating any exceptions
    populate_instance(msg, inst)


class ActionClientHandler(Generic[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT]):
    def __init__(
        self,
        action: str,
        action_type: str,
        success_callback: Callable[[dict[str, Any]], None],
        error_callback: Callable[[Exception], None],
        feedback_callback: Callable[[FeedbackMessage[ROSActionFeedbackT]], None] | None,
        node_handle: Node,
        server_timeout_time: float = 1.0,
    ) -> None:
        """
        Create a client handler for the specified action.

        Use start() to start in a separate thread or run() to run in this thread.

        :param action: The name of the action to execute.
        :param action_type: The type of the action to execute.
        :param args: Arguments to pass to the action. Can be an ordered list, or a dict of
            name-value pairs. Anything else will be treated as though no arguments were provided
            (which is still valid for some kinds of actions)
        :param success_callback: A callback to call with the JSON result of the service call
        :param error_callback: A callback to call if an error occurs. The callback will be passed
            the exception that caused the failure
        :param node_handle: A ROS 2 node handle to call services
        """
        self.action = action
        self.action_type = action_type
        self.success_callback = success_callback
        self.error_callback = error_callback
        self.feedback_callback = feedback_callback
        self.node_handle = node_handle
        self.server_timeout_time = server_timeout_time
        self.goal_handle: ClientGoalHandle | None = None
        self.goal_canceled = False
        self.result = None
        self.action_client = ActionClient(self.node_handle, get_action_class(self.action_type), self.action)

    def send_goal(
        self,
        args: list | dict[str, Any] | None = None,
    ) -> Future | None:
        inst = cast("ROSActionGoalT", get_action_goal_instance(self.action_type))

        # Populate the instance with the provided args
        args_to_action_goal_instance(inst, args)

        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout_time):
            msg = "No action server available"
            self.error_callback(Exception(msg))
            self.goal_handle = None
            return None
        send_goal_future : Future = self.action_client.send_goal_async(inst, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_cb)
        return send_goal_future

    def get_result_cb(self, future: Future) -> None:
        self.success_callback(extract_values(future.result()))
        self.goal_handle = None
        self.action_client.destroy()

    def goal_response_cb(self, future: Future) -> None:
        self.goal_handle = future.result()
        assert self.goal_handle is not None
        if not self.goal_handle.accepted:
            msg = "Action goal was rejected"
            self.error_callback(Exception(msg))
            self.goal_handle = None
            return
        result_future: Future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_cb)

    def goal_cancel_cb(self, _: Future) -> None:
        self.error_callback(Exception(f"Action goal was canceled"))
        self.goal_canceled = True
        self.goal_handle = None
        self.action_client.destroy()

    def cancel_goal(self) -> None:
        if self.goal_handle:
            cancel_goal_future = self.goal_handle.cancel_goal_async()
            cancel_goal_future.add_done_callback(self.goal_cancel_cb)
