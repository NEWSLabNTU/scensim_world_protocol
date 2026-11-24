"""The descriptor carries `StepReply`'s tick stamps and `HelloReply.boot_id` (scensim issue 135
step 3), and a reply built without them (this shell does not stamp) reads as 0 and empty."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scensim_world_server import protocol as pb  # noqa: E402


class TickStamps(unittest.TestCase):
    def test_the_fields_are_in_the_descriptor(self):
        f = pb.StepReply.DESCRIPTOR.fields_by_name
        self.assertEqual(f["tick_start_mono_ns"].number, 9)
        self.assertEqual(f["tick_return_mono_ns"].number, 10)
        self.assertEqual(pb.HelloReply.DESCRIPTOR.fields_by_name["boot_id"].number, 10)

    def test_unstamped_reads_zero_and_stamped_round_trips(self):
        bare = pb.StepReply.FromString(pb.StepReply(frame=4).SerializeToString())
        self.assertEqual((bare.tick_start_mono_ns, bare.tick_return_mono_ns), (0, 0))
        r = pb.StepReply(frame=4, tick_start_mono_ns=5_000_000_000,
                         tick_return_mono_ns=5_001_200_000)
        back = pb.StepReply.FromString(r.SerializeToString())
        self.assertEqual((back.tick_start_mono_ns, back.tick_return_mono_ns),
                         (5_000_000_000, 5_001_200_000))
        self.assertEqual(pb.HelloReply.FromString(b"").boot_id, "")


if __name__ == "__main__":
    unittest.main()
