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

import fnmatch
from collections.abc import Callable
from functools import partial
from typing import TYPE_CHECKING, Any

from action_msgs.msg import GoalStatus

from rosbridge_library.capability import Capability
from rosbridge_library.internal.actions import ActionClientHandler
from rosbridge_library.internal.message_conversion import extract_values

if TYPE_CHECKING:
    from rosbridge_library.internal.type_support import FeedbackMessage, ROSMessage
    from rosbridge_library.protocol import Protocol


class SendActionGoal(Capability):
    send_action_goal_msg_fields = (
        (True, "action", str),
        (True, "action_type", str),
        (False, "fragment_size", (int, type(None))),
        (False, "compression", str),
    )
    cancel_action_goal_msg_fields = ((True, "action", str),)

    client_handler_list: dict[str, ActionClientHandler]
    action_goal_queue: list[dict] = []

    parameter_names = ("actions_glob", "send_action_goals_in_new_thread")

    actions_glob: list[str] | None = None
    send_action_goals_in_new_thread: bool = False

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        self.client_handler_list = {}

        # Register the operations that this capability provides
        protocol.register_operation(
            "send_action_goal", lambda msg: self.add_task_to_executor(msg, self.send_action_goal)
        )
        protocol.register_operation(
            "cancel_action_goal",
            lambda msg: self.add_task_to_executor(msg, self.cancel_action_goal),
        )

    def add_task_to_executor(self, msg: dict, callback: Callable[[dict], None]) -> None:
        if self.protocol.node_handle.executor:
            self.protocol.node_handle.executor.create_task(partial(callback, msg))
        else:
            self.protocol.node_handle.get_logger().error("Failed sending, executor is None")

    def send_action_goal(self, message: dict) -> None:
        if self.send_action_goals_in_new_thread or not self.client_handler_list:
            # Pull out the ID
            cid: str | None = message.get("id")
            if cid is None:
                raise ValueError("Action goal must have an ID")
            # Typecheck the args
            self.basic_type_check(message, self.send_action_goal_msg_fields)

            # Extract the args
            action: str = message["action"]
            action_type: str = message["action_type"]
            fragment_size: int | None = message.get("fragment_size")
            compression: str = message.get("compression", "none")
            args: list | dict[str, Any] = message.get("args", [])

            if self.actions_glob is not None:
                self.protocol.log(
                    "debug", f"Action security glob enabled, checking action: {action}"
                )
                match = False
                for glob in self.actions_glob:
                    if fnmatch.fnmatch(action, glob):
                        self.protocol.log(
                            "debug",
                            f"Found match with glob {glob}, continuing sending action goal...",
                        )
                        match = True
                        break
                if not match:
                    msg = f"No match found for action, cancelling sending action goal for: {action}"
                    self.protocol.log(
                        "warn",
                        msg,
                    )
                    self._failure(cid, action, Exception(msg))
                    return
            else:
                self.protocol.log(
                    "debug", "No action security glob, not checking sending action goal."
                )

            # Create the callbacks
            success_callback = partial(self._success, cid, action, fragment_size, compression)
            feedback_callback = (
                partial(self._feedback, cid, action) if message.get("feedback", False) else None
            )
            error_callback = partial(self._failure, cid, action)

            self.client_handler_list[cid] = ActionClientHandler(
                action,
                action_type,
                success_callback,
                error_callback,
                feedback_callback,
                self.protocol.node_handle,
            )
            self.client_handler_list[cid].send_goal(args)
        else:
            self.action_goal_queue.append(message)

    def cancel_action_goal(self, message: dict) -> None:
        # Extract the args
        cid = message.get("id")
        action = message["action"]

        # Typecheck the args
        self.basic_type_check(message, self.cancel_action_goal_msg_fields)

        # Pull out the ID
        # Check for deprecated action ID, eg. /rosbridge/topics#33
        cid = extract_id(action, cid)

        # Cancel the action
        if cid in self.client_handler_list:
            self.client_handler_list[cid].cancel_goal()

    def _success(
        self,
        cid: str | None,
        action: str,
        _fragment_size: int | None,
        _compression: str,
        message: dict,
    ) -> None:
        outgoing_message = {
            "op": "action_result",
            "action": action,
            "values": message["result"],
            "status": message["status"],
            "result": True,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)
        self.client_handler_list.pop(cid, None)
        if self.action_goal_queue:
            self.send_action_goal(self.action_goal_queue.pop(0))

    def _failure(self, cid: str | None, action: str, exc: Exception) -> None:
        self.protocol.log("error", f"send_action_goal {type(exc).__name__}: {cid}")
        # send response with result: false
        outgoing_message = {
            "op": "action_result",
            "action": action,
            "values": str(exc),
            "status": GoalStatus.STATUS_UNKNOWN,
            "result": False,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        self.protocol.send(outgoing_message)
        self.client_handler_list.pop(cid, None)
        if self.action_goal_queue:
            self.send_action_goal(self.action_goal_queue.pop(0))

    def _feedback(self, cid: str | None, action: str, message: FeedbackMessage[ROSMessage]) -> None:
        outgoing_message = {
            "op": "action_feedback",
            "action": action,
            "values": extract_values(message.feedback),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)


def trim_action_name(action: str) -> str:
    if "#" in action:
        return action[: action.find("#")]
    return action


def extract_id(action: str, cid: str | None) -> str | None:
    if cid is not None:
        return cid
    if "#" in action:
        return action[action.find("#") + 1 :]
    return None
