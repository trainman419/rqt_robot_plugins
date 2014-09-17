# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
# Author: Isaac Saito, Ze'ev Klapow

from diagnostic_msgs.msg import DiagnosticStatus
from python_qt_binding.QtGui import QColor, QIcon
import rospy

# TODO: Utils and common configs are mixed in this class.

# Instantiating icons that show the device status.
_ERR_ICON = QIcon.fromTheme('dialog-error')
_WARN_ICON = QIcon.fromTheme('dialog-warning')
_OK_ICON = QIcon.fromTheme('emblem-default')
# Added following this QA thread http://goo.gl/83tVZ
_STALE_ICON = QIcon.fromTheme('dialog-question')

_LEVEL_TO_ICON = {0: _OK_ICON, 1: _WARN_ICON, 2: _ERR_ICON, 3: _STALE_ICON}

_LEVEL_TO_COLOR = {0: QColor(85, 178, 76),  # green
                   1: QColor(222, 213, 17), # yellow
                   2: QColor(178, 23, 46),  # red
                   3: QColor(40, 23, 176)   # blue
                   }

_LEVEL_TO_TEXT = { 0: "OK", 1: "WARNING", 2: "ERROR", 3: "STALE" }

def level_to_icon(level):
    if level in _LEVEL_TO_ICON:
        return _LEVEL_TO_ICON[level]
    else:
        return _ERR_ICON

def level_to_color(level):
    if level in _LEVEL_TO_COLOR:
        return _LEVEL_TO_COLOR[level]
    else:
        return _LEVEL_TO_COLOR[2]

def level_to_text(level):
    if level in _LEVEL_TO_TEXT:
        return _LEVEL_TO_TEXT[level]
    else:
        return "UNKNOWN(%d)" % ( level )

_DICTKEY_TIMES_ERROR = 'times_errors'
_DICTKEY_TIMES_WARN = 'times_warnings'
_DICTKEY_INDEX = 'index'
_DICTKEY_STATITEM = 'statitem'

def update_status_images(diagnostic_status, statusitem):
    """
    Taken from robot_monitor.robot_monitor_panel.py.

    :type status: DiagnosticStatus
    :type node: StatusItem
    :author: Isaac Saito
    """

    name = diagnostic_status.name
    if (name is not None):
        level = diagnostic_status.level
        if (diagnostic_status.level != statusitem.last_level):
            statusitem.setIcon(0, level_to_icon(level))
            statusitem.last_level = level
            return

def get_resource_name(status_name):
    """
    Get resource name from path

    :param: status_name is a string that may consists of status names that
            are delimited by slash.
    :rtype: str
    """
    name = status_name.split('/')[-1]
    rospy.logdebug(' get_resource_name name = %s', name)
    return name

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

def get_color_for_message(msg):
    """
    Get the overall (worst) color for a DiagnosticArray
    :param msg: DiagnosticArray
    """

    level = 0
    min_level = 255

    lookup = {}
    for status in msg.status:
        lookup[status.name] = status

    # WHY?
    names = [status.name for status in msg.status]
    names = [name for name in names
             if len(get_parent_name(name)) == 0]
    for name in names:
        status = lookup[name]
        if (status.level > level):
            level = status.level
        if (status.level < min_level):
            min_level = status.level

    # Stale items should be reported as errors unless all stale
    if (level > 2 and min_level <= 2):
        level = 2

    rospy.logdebug(' get_color_for_message color lv=%d', level)
    return level_to_color(level)

def get_correspondent(key, list_statitem):
    """
    :type key: String.
    :type list_statitem: DiagnosticsStatus
    :rtype: StatusItem
    """
    names_from_list = [get_resource_name(status.name)
                       for status in list_statitem]
    key_niced = get_resource_name(key)
    index_key = -1
    statitem_key = None
    if key_niced in names_from_list:
        index_key = names_from_list.index(key_niced)
        statitem_key = list_statitem[index_key]
        rospy.logdebug(' get_correspondent index_key=%s statitem_key=%s',
                      index_key, statitem_key)
    return {_DICTKEY_INDEX: index_key,
            _DICTKEY_STATITEM: statitem_key}

def get_children(name, diag_array):
    """
    :type msg: DiagnosticArray
    :rtype: DiagnosticStatus[]
    """

    ret = []
    for k in diag_array.status:  # k is DiagnosticStatus.
        if k.name.startswith(name):  # Starting with self.name means k
                                    # is either top/parent node / its child
            if not k.name == name:  # Child's name must be different
                                        # from that of the top/parent node.
                ret.append(k)
    return ret

def get_status_by_name(msg, name):
    for status in msg.status:
        if status.name == name:
            return status
    return None
