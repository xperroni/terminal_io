#! /usr/bin/env python

from actionlib import SimpleActionClient
from rospy import resolve_name

from terminal_io.msg import ExchangeAction, ExchangeGoal


# Action invocation modes.
(KEY, LINE, PRINT) = range(3)


class Terminal(object):
    def __init__(self, name='terminal'):
        self.client = SimpleActionClient(resolve_name(name), ExchangeAction)
        self.client.wait_for_server()

    def __del__(self):
        self.close()

    def close(self):
        del self.client

    def send(self, mode, text, handler=None, block=True):
        goal = ExchangeGoal()
        goal.mode = mode
        goal.text = text
        self.client.send_goal(goal, feedback_cb=handler)

        if block:
            self.client.wait_for_result()

    def write(self, text):
        self.send(PRINT, text)

    def prompt(self, text):
        self.send(LINE, text)
        return self.client.get_result().line

    def get(self, text, escape, handler, block=True):
        callback = lambda feedback: handler(feedback.key)
        self.send(KEY, escape + text, callback, block)
