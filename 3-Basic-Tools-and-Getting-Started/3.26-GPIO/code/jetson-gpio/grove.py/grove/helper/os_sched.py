#!/usr/bin/env python
#
# This is the library for Grove Base Hat.
#
# OS Scheduler Classes
#
'''
provide functions to promote process real-time priority or change back to default

## License

The MIT License (MIT)

Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
Copyright (C) 2018  Seeed Technology Co.,Ltd. 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''
from __future__ import print_function
import sys

SCHED_OTHER  = 0
SCHED_FIFO   = 1
SCHED_RR     = 2
SCHED_BATCH  = 3
SCHED_IDLE   = 5


_os_import = True
max_param = None
if sys.version_info >= (3, 3):
    import os as osm

    max_priority = osm.sched_get_priority_max(SCHED_FIFO)
    max_param  = osm.sched_param(max_priority)
    norm_param = osm.sched_param(0)
else:
    try:
        import ctypes
        import ctypes.util
        osm = ctypes.cdll.LoadLibrary(ctypes.util.find_library('c'))
    except Exception:
        print("error: module %s unusable" % __name__);
        _os_import = False

if not max_param and _os_import:
    class _sched_param(ctypes.Structure):
        _fields_ = [ ('sched_priority', ctypes.c_int) ]
    max_param_c = _sched_param()
    max_priority = osm.sched_get_priority_max(SCHED_FIFO)
    # print("max priority = %d" % max_priority)
    max_param_c.sched_priority = max_priority
    max_param = ctypes.byref(max_param_c)

    norm_param_c = _sched_param()
    norm_param_c.sched_priority = 0
    norm_param = ctypes.byref(norm_param_c)


def set_max_priority():
    "Set current process to highest priority"
    if not _os_import:
        return False
    osm.sched_setscheduler(0, SCHED_FIFO, max_param)

def set_default_priority():
    "Set current process to default priority"
    if not _os_import:
        return False
    osm.sched_setscheduler(0, SCHED_OTHER, norm_param)


'''
class Sched(object):

    def __init__(self):
        "Initialize Sched object"
'''


if __name__ == '__main__':
    import time
    set_max_priority()
    time.sleep(1)
    set_default_priority()

