#!/usr/bin/env python

from smach import State


class ConfirmState(State):
    def __init__(self, question, resp=True):
        State.__init__(self, outcomes=['yes', 'no'])
        self.resp = 'yes' if resp else 'no'
        if resp:
            self.prompt = '%s [%s]|%s: ' % (question, 'y', 'n')
        else:
            self.prompt = '%s [%s]|%s: ' % (question, 'n', 'y')

    def execute(self, userdata):
        while True:
            ans = raw_input(self.prompt)
            if not ans:
                return self.resp
            if ans not in ['y', 'Y', 'n', 'N']:
                print 'Please enter "y" or "n".'
                continue
            if ans == 'y' or ans == 'Y':
                return 'yes'
            if ans == 'n' or ans == 'N':
                return 'no'
