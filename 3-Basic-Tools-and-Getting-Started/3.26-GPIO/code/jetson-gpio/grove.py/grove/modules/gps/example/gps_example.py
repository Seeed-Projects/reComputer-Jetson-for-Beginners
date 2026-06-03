"""
License
The MIT License (MIT)

Copyright (C) 2020 Ville Laine, Aikuiskoulutus Taitaja

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

NOTE!!
README.md for setting the Raspberry Pi's serial line!!!
`/dev/ttyS0` (aka mini UART) is too slow!!!
"""
import threading
import time
from grove.modules.gps import GPS


#############################################################
# Thread that updates data from receiver to inner variables #
#############################################################
class UpdateGPSData(threading.Thread):
    def __init__(self, gps_ins, name="UpdateGPS", daemon=False):
        """ constructor, setting initial variables """
        self._stop_event = threading.Event()
        self._sleep_period = 0.05
        self.gps = gps_ins
        self.error = False
        self._fail_count = 0
        self._max_fail_count = 10
        self._max_fail_sleep = 3
        threading.Thread.__init__(self, name=name, daemon=daemon)

    def run(self):
        print("### Starting gps update Thread! ###")

        """ Main Control loop """
        while not self._stop_event.isSet():
            # Update gps data if there is data
            # Query from serial interface if gps receiver has sent data with new_data()
            if self.gps.new_data():
                thread_lock.acquire()
                if self.gps.read():
                    # If everything went well reset fail status
                    self._fail_count = 0
                thread_lock.release()
            else:
                # And wait for little while if data read failed before trying next time
                self._fail_count += 1
                time.sleep(min(self._sleep_period * self._fail_count, self._max_fail_sleep))

            if self._fail_count > self._max_fail_count:
                print("Data update failed last %d times!!\nSleeping %.3f" %
                      (self._fail_count, min(self._sleep_period * self._fail_count, self._max_fail_sleep)))
                self.error = True
            else:
                self.error = False

        print("### Exiting gps update Thread! ###")

    # Define custom join object to set the exit flag to the run object
    def join(self, timeout=None):
        """ Stop the thread """
        self._stop_event.set()
        threading.Thread.join(self, timeout)


#############################
# Thread that displays data #
#############################
class PrintGPSData(threading.Thread):
    def __init__(self, gps_ins, name="PrintGPSData", daemon=False):
        """ constructor, setting initial variables """
        self._stop_event = threading.Event()
        self._sleep_period = 1
        self.gps = gps_ins
        threading.Thread.__init__(self, name=name, daemon=daemon)

    def run(self):
        print("### Starting gps print Thread! ###")

        """ Main Control loop """
        while not self._stop_event.isSet():
            thread_lock.acquire()
            print("\n%s" % self.gps.utc_datetime)
            print("Satellites in view:", gps.GSV_satellites_in_view)
            print("Satellites used:", gps.GGA_satellites_used)
            print("Position:", self.gps.location_DDM)
            print("Speed: %s and Course: %s" % (gps.RMC_speed_over_ground, gps.RMC_course_over_ground))

            # Release the lock and then sleep
            thread_lock.release()
            time.sleep(self._sleep_period)

        print("### Exiting gps print Thread! ###")

    # Define custom join object to set the exit flag to the run object
    def join(self, timeout=None):
        """ Stop the thread """
        self._stop_event.set()
        threading.Thread.join(self, timeout)


# MAIN THREAD #
if __name__ == "__main__":
    # Connect to gps receiver
    gps = GPS('/dev/ttyAMA0', 9600, 1)

    # Create ThreadLock for synchronization
    # Acquiring the lock will block from other threads from processing before lock is released again
    # thread_lock.acquire()
    # thread_lock.release()
    thread_lock = threading.Lock()

    # Create instances of update and print threads
    # Threads can be made as daemonic threads =
    # When all non-daemonic threads are exited all daemonic threads are exited too
    # but we control the closing individually with join() and with the help of an thread event
    gps_update_thread = UpdateGPSData(gps, daemon=False)
    gps_print_thread = PrintGPSData(gps, daemon=False)

    try:
        # Start the threads
        gps_update_thread.start()
        gps_print_thread.start()

        # And wait for Keyboard Interrupt
        while True:
            if gps_update_thread.error:
                # Stop the print thread because there is no new data to be printed
                gps_print_thread.join()
            else:
                if not gps_print_thread.is_alive():
                    # Create and start print thread again
                    gps_print_thread = PrintGPSData(gps, daemon=False)
                    gps_print_thread.start()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        # Stop the threads
        gps_print_thread.join()
        gps_update_thread.join()

    print("### Exiting Main Thread! ###")
