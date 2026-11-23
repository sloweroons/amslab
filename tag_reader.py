#!/usr/bin/python3
# -*- coding: utf-8 -*-
''' NFC-tag reader

This program publishes a ROS message with the ID of the tag whenever a new tag is detected with MFRC-522 module that is attached to the Raspberry Pi. 

# Installation

This program requires an enabled SPI interface on the Raspberry Pi and Python libraries for that interface.

```bash
#Use sudo raspi-config to enable SPI interface or:
sudo sed -i '/^[# ]*dtparam=spi=.*/d' /boot/config.txt
echo 'dtparam=spi=on' | sudo tee -a /boot/config.txt

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install python3-dev python3-pip
sudo pip3 install RPi.GPIO

cd
git clone https://github.com/lthiery/SPI-Py.git

cd ~/SPI-Py
sudo python3 setup.py install

#cd
#git clone https://github.com/pimylifeup/MFRC522-python.git
#git clone https://github.com/naleefer/MFRC522-python.git
```

# Subscribing to the tag topic

To subscribe to the tag topic, add the following lines to your program.

Add the following line to the import section:
```python
from amsagv_msgs.msg import TagStamped
from amsagv.ams import qos_event
```

Add the following line to the `__init__` method of your class (`node` is your rclpy node):
```python
self._sub_tag = node.create_subscription(TagStamped, 'tag', self._handle_tag, qos_event)
```

Add the following method to your class:
```python
def _handle_tag(self, msg):
    print('Tag data: {}'.format(msg.tag.id))
```
'''

import time

from amsagv import mfrc522
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_event
from amsagv_msgs.msg import TagStamped



class TagReader(object):
    def __init__(self):
        self._reader = None

    def __enter__(self):
        # spd: SPI clock [Hz]; gain: max receiver gain; timeout: how long the reader waits for
        # a tag reply [s]; poll_sleep: sleep between status reads while waiting [s] (0 = busy wait)
        self._reader = mfrc522.MFRC522(spd=1000000, gain=mfrc522.MFRC522.GAIN_48DB, timeout=0.005,
                                       poll_sleep=0.0005)
        return self

    def __exit__(self, *args, **kwargs):
        self._reader.close_mfrc522()

    def version(self):
        return self._reader.version()

    def read(self):
        '''Poll once; return the tag ID (int) or None if no tag is in the field.'''
        return TagReader.uid_to_num(self._reader.read_uid())

    @staticmethod
    def uid_to_num(uid):
        if uid is None or len(uid) != 5:
            return None
        else:
            n = 0
            for i in range(5):
                n = n * 256 + uid[i]
            return n



class TagTracker(object):
    '''Decides when a detection is a new tag event.

    A tag is published when it is seen and it is not the tag that is currently
    present. The tag stays present until it has not been read for `hold_time`
    seconds, so missed polls while the tag is still under the antenna do not
    cause duplicate messages.
    '''

    def __init__(self):
        self.current = None
        self.last_seen = None

    def update(self, tag_id, now):
        '''Return True if `tag_id` (read at time `now`, or None) should be published.'''
        hold_time = 0.5 # a tag counts as still present (and is not published again) until it has not been seen for this long [s] (default 0.5)
        if self.current is not None and now - self.last_seen > hold_time:
            self.current = None
        if tag_id is None:
            return False
        new = tag_id != self.current
        self.current = tag_id
        self.last_seen = now
        return new



def main():
    rclpy.init()
    node = Node('tag_reader')

    period = 0.01 # polling period [s] (default 0.01 = 100 Hz: robust detection with ~3 mm travel per poll at 0.3 m/s, ~17 % of one CPU core; 0 = as fast as possible)

    try:
        with TagReader() as reader:
            ver = reader.version()
            #if ver in (0x00, 0xFF):
            #    node.get_logger().error('MFRC522 not responding (version 0x{:02X})'.format(ver))
            #else:
            #    node.get_logger().info('MFRC522 version 0x{:02X}'.format(ver))

            # Tag publisher
            pub = node.create_publisher(TagStamped, 'tag', qos_event)

            # Tag message
            msg = TagStamped()

            tracker = TagTracker()
            t_spin = time.monotonic()
            while rclpy.ok():
                t = time.monotonic()
                stamp = node.get_clock().now()
                tag_id = reader.read()
                if tracker.update(tag_id, t):
                    msg.header.stamp = stamp.to_msg()
                    msg.tag.id = int(tag_id)
                    pub.publish(msg)
                    #node.get_logger().debug('Tag {}'.format(tag_id))

                now = time.monotonic()
                if now - t_spin >= 0.1:  # serve parameter services etc.
                    rclpy.spin_once(node, timeout_sec=0.0)
                    t_spin = now
                if period > 0.0:
                    dt = period - (time.monotonic() - t)
                    if dt > 0.0:
                        time.sleep(dt)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
