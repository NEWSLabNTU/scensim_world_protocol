"""The descriptor carries `EntityState.applied_control` (scensim issue 135), and a state this
shell builds without it (Isaac) reads as unset, not as a zero control."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scensim_world_server import protocol as pb  # noqa: E402


class AppliedControl(unittest.TestCase):
    def test_the_field_and_its_message_are_in_the_descriptor(self):
        field = pb.EntityState.DESCRIPTOR.fields_by_name["applied_control"]
        self.assertEqual(field.number, 6)
        self.assertEqual(field.message_type.name, "AppliedControl")
        names = [f.name for f in pb.AppliedControl.DESCRIPTOR.fields]
        self.assertEqual(names, ["throttle", "brake", "steer", "hand_brake", "reverse",
                                 "manual_gear_shift", "gear"])

    def test_a_state_without_it_reads_unset_and_one_with_it_round_trips(self):
        bare = pb.EntityState.FromString(pb.EntityState(handle=3).SerializeToString())
        self.assertFalse(bare.HasField("applied_control"))
        s = pb.EntityState(handle=3)
        s.applied_control.throttle = 0.25
        s.applied_control.steer = -0.5
        back = pb.EntityState.FromString(s.SerializeToString())
        self.assertTrue(back.HasField("applied_control"))
        self.assertEqual((back.applied_control.throttle, back.applied_control.steer),
                         (0.25, -0.5))


if __name__ == "__main__":
    unittest.main()
