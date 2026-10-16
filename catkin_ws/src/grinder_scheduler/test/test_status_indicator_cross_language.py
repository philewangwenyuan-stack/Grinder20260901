"""Use the production C++ parser against Python wire frames (host executable)."""
import os
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))
from grinder_scheduler.status_indicator import LightState, envelope, state_frame, valid_ack


@unittest.skipUnless(os.environ.get("INDICATOR_PROTOCOL_TEST_EXE"), "compile firmware/status_strip/test/protocol_host.cpp first")
class CrossLanguageTest(unittest.TestCase):
    def execute(self, frames):
        result = subprocess.run([os.environ["INDICATOR_PROTOCOL_TEST_EXE"], "stdin"],
                                input=b"".join(frames), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                check=True, timeout=10)
        lines = result.stdout.splitlines(keepends=True)
        self.assertEqual(b"receiver self-tests passed", lines[0].strip())
        return [line.replace(b"\r\n", b"\n") for line in lines[1:]]

    def test_python_frames_and_cpp_ack_match(self):
        session = "0123456789abcdef"
        frames = [state_frame(session, int(state) + 1, state) for state in LightState]
        frames.append(state_frame(session, len(LightState) + 1, LightState.READY, clear=True))
        replies = self.execute(frames)
        self.assertEqual(len(frames), len(replies))
        for sequence, reply in enumerate(replies, 1):
            self.assertTrue(valid_ack(reply, session, sequence))

    def test_firmware_rejects_corruption_and_out_of_range_fields(self):
        session = "0123456789abcdef"
        frames = [
            envelope("S1,{},1,1,103,0".format(session)),
            envelope("S1,{},1,7,51,1".format(session)),
            envelope("S1,{},4294967296,1,51,0".format(session)),
            envelope("S1,{},0,1,51,0".format(session)),
            envelope("S1,{},1,9,51,0".format(session)),
            envelope("S1,{},1,1,51,0,extra".format(session)),
            envelope("S1,{},1,1,51,0".format(session)).replace(b",1,1,", b",1,2,"),
        ]
        self.assertEqual([b"REJECTED\n"] * len(frames), self.execute(frames))


if __name__ == "__main__":
    unittest.main()
