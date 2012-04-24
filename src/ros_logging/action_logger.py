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

# Defines ActionLogger class 

import os
import pymongo as pm
import pymongo.binary as binary
import time
import rospy
import re
import ros_logging.action_subscriber as sub
import ros_actions as act

ACTION_TYPE_COLL = 'action_logging_action_types'
ACTION_COLL_NAME = 'action_log'

class ActionLogger(object):
    """
    Maintains state of the action logger.  Individual action logging is done
    by the ActionSubscriber class.
    """

    def __init__(self, db, master):
        """
        Initialize with a pymongo db object and an xmlrpc
        connection to the ros master
        """
        self.db = db
        self.master = master
        self.action_type_coll = db[ACTION_TYPE_COLL]

        # Keep local copy of seen action topics, to reduce querying
        self.actions = {}

    def update(self):
        resp = self.master.getPublishedTopics('action_logger', '')
        if resp[0]!=1:
            rospy.logerr("Received response {0} from master; skipping".\
                         format(resp))
            return
        topics = resp[2]
        # Now guaranteed we have a valid topic list from master
        
        for topic, msg_type in topics:
            res = re.match('(.*)/result$', topic)
            if res is None:
                continue
            # Now guaranteed topic ends with '/result'
            
            name = act.normalize_name(res.group(1))
            if name in self.actions:
                continue
            
            # Now guaranteed this is a previously unseen action
            # Figure out the package and the type of the result and goal
            res2 = re.match('(.*)/(.*)$', msg_type)
            pkg = res2.group(1)
            result_suffix='Result'
            goal_suffix='Goal'
            action_type = res2.group(2)[:-len(result_suffix)]

            # Add an entry to the list of action names and types if it's not
            # already there
            self.actions[name] = (pkg, action_type)
            doc = self.action_type_coll.find_one({'name': name})
            if doc:
                if doc['pkg']!=pkg or doc['type']!=action_type:
                    msg = "Existing type {0}/{1} for {2} didn't match {3}/{4};"\
                          "terminating".format(doc['pkg'], doc['type'], name,
                                               pkg, action_type)
                    raise RuntimeError(msg)
                action_id = doc['_id']
            else:
                entry = {'name': name, 'pkg': pkg, 'type': action_type}
                action_id = self.action_type_coll.insert(entry)

            # Set up subscription
            coll_name = 'action_logger_'+name
            self.actions[name] = sub.ActionSubscriber(name, self.db[ACTION_COLL_NAME],
                                                      pkg, action_type, action_id)
