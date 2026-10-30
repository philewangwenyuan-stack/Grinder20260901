"""Generated ROS numpy serialization, no ROS master or physical hardware."""
import io
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from grinder_scheduler.map_grid_utils import occupancy_grid_array

try:
    import rospy
    from rospy.numpy_msg import numpy_msg
    from nav_msgs.msg import OccupancyGrid
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, 'Requires generated ROS Noetic message classes; run on board or ROS environment')
class NumpySerializationTest(unittest.TestCase):
    def test_numpy_message_wire_compatible_and_received_data_readonly(self):
        cls = numpy_msg(OccupancyGrid)
        sent = cls()
        sent.header.frame_id = 'map'
        sent.header.stamp = rospy.Time(12, 345)
        sent.info.width, sent.info.height, sent.info.resolution = 3, 2, .05
        sent.info.origin.position.x, sent.info.origin.position.y = -1., 2.
        sent.info.origin.orientation.w = 1.
        original = np.array([[-1, 0, 100], [99, 1, 100]], np.int8)
        sent.data = np.frombuffer(original.tobytes(), dtype=np.int8)
        stream = io.BytesIO()
        sent.serialize(stream)
        wire = stream.getvalue()
        regular = OccupancyGrid().deserialize(wire)
        self.assertEqual(tuple(original.ravel()), tuple(regular.data))
        received = cls().deserialize(wire)
        self.assertFalse(received.data.flags.writeable)
        grid = occupancy_grid_array(received)
        self.assertTrue(np.shares_memory(grid, received.data))
        np.testing.assert_array_equal(grid, original)
        stream = io.BytesIO()
        received.serialize(stream)
        self.assertEqual(wire, stream.getvalue())
        self.assertEqual((-1., 2., 12, 345), (received.info.origin.position.x,
                         received.info.origin.position.y, received.header.stamp.secs, received.header.stamp.nsecs))


if __name__ == '__main__':
    unittest.main()
