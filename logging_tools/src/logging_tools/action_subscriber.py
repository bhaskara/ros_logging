# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Author: Bhaskara Marthi

# Subscribes to a ros action's goal and results

import threading
import rospy
import re
import time
import pymongo.binary as binary
import roslib
import ros_actions as act
from cStringIO import StringIO

class ActionSubscriber(object):
    
    def __init__(self, name, coll, pkg, action_type, action_id):
        self.coll = coll
        self.pkg = pkg
        self.action_type = action_type
        self.goal_class, self.result_class = act.get_classes(pkg, action_type)
        self.action_id = action_id
        
        self.lock = threading.Lock()

        self.goal_id = None
        self.goal_sub = rospy.Subscriber(name+'/goal', rospy.AnyMsg,
                                         self.handle_goal)
        self.result_sub = rospy.Subscriber(name+'/result', rospy.AnyMsg,
                                           self.handle_result)


    def handle_goal(self, raw):
        print "Received goal"
        with self.lock:
            t = time.time()
            msg = self.goal_class()
            msg.deserialize(raw._buff)

            # rospy.loginfo("Received goal {0}".format(self.msg))

            # Write to db
            item = {'goal_id': msg.goal_id.id, 'blob': binary.Binary(raw._buff),
                    'type': 'goal', 'time': t,
                    'action_id': self.action_id}
            self.coll.insert(item)
            print "Inserted ", item

    def handle_result(self, raw):
        print "received result"
        with self.lock:
            # Get the message object
            t = time.time()
            msg = self.result_class()
            msg.deserialize(raw._buff)
            # rospy.loginfo("Received result {0}".format(msg))
            
            # Write to db
            item = {'type': 'result', 'time': t, 'status': msg.status.status,
                    'goal_id': msg.status.goal_id.id, 'action_id': self.action_id}
            self.coll.insert(item)
            print "Inserted ", item

            # rospy.loginfo("Done handling result")
            

            
