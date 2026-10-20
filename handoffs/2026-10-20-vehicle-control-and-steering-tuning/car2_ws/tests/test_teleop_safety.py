"""Regression tests for actual teleop logic; no USB or keyboard hooks opened."""
import importlib.util
import pathlib
import sys
import time
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop"))
keys = types.SimpleNamespace(space="SPACE", left="LEFT", right="RIGHT", esc="ESC", f2="F2")
keyboard = types.SimpleNamespace(Key=keys, KeyCode=type("KeyCode", (), {}))
spec = importlib.util.spec_from_file_location("tested_teleop", ROOT / "teleop/keyboard_teleop_wasd_guarded.py")
teleop = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {"pynput": types.SimpleNamespace(keyboard=keyboard)}):
    spec.loader.exec_module(teleop)


class TeleopSafety(unittest.TestCase):
    def setUp(self):
        self.act = mock.Mock()
        self.app = teleop.WasdTeleop(self.act, 50, 1, teleop.DEFAULT_SPEED_LEVELS,
                                     0.05, teleop.DEFAULT_STEER_TRIM, 1, 0.1)

    def ready(self):
        self.app.telem = dict(tr=1, sr=1, br=1, sh=1, sf=0, tm=0)
        self.app.telem_at = time.monotonic()

    def test_startup_has_no_left_trim(self):
        self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))
        self.assertEqual(teleop.DEFAULT_STEER_TRIM, 0)

    def test_estop_suppresses_held_steering_and_explicit_trim(self):
        self.app.steer_trim = -0.4
        self.app._pressed.add("a")
        for _ in range(20):
            self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))

    def test_estop_cannot_clear_with_missing_can_or_reference(self):
        for missing in ("tr", "sr", "br", "sh"):
            self.ready()
            self.app.telem[missing] = 0
            self.app.on_press("e")
            self.app.on_release("e")
            self.assertTrue(self.app.estop)

    def test_clear_estop_does_not_enable_drive(self):
        self.ready()
        self.app.on_press("e")
        self.assertFalse(self.app.estop)
        self.assertFalse(self.app.drive_enabled)
        self.assertEqual(self.app._compute()[1:3], (0, "N"))

    def test_z_reference_is_only_sent_in_estop(self):
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()
        self.app.on_release("z")
        self.ready()
        self.app.on_press("e")
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()

    def test_manual_reference_toggle_blocks_z_and_ignores_key_repeat(self):
        self.app.on_press(keys.f2)
        self.app.on_press(keys.f2)  # OS key repeat must not re-enable it.
        self.assertFalse(self.app.manual_reference_enabled)
        self.app.on_press("z")
        self.app.on_release("z")
        self.act.reference_steering.assert_not_called()
        self.app.on_release(keys.f2)
        self.app.on_press(keys.f2)
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()

    def test_manual_reference_toggle_cannot_change_while_active(self):
        self.ready()
        self.app.estop = False
        self.app.on_press(keys.f2)
        self.assertTrue(self.app.manual_reference_enabled)
        self.app.on_press("z")
        self.act.reference_steering.assert_not_called()

    def test_disabled_manual_reference_explains_reference_inhibit(self):
        self.ready()
        self.app.telem["sh"] = 0
        self.app.manual_reference_enabled = False
        self.assertEqual(self.app._inhibit_reason(), "Z-OFF (F2)")
        self.app.on_press("e")
        self.assertTrue(self.app.estop)
        self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))

    def test_lost_feedback_latches_estop_and_stops_all_commands(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app._pressed.update(("w", "d"))
        self.app.telem["br"] = 0
        self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))
        self.assertFalse(self.app.drive_enabled)

    def test_frozen_master_telemetry_cannot_keep_driving(self):
        self.ready()
        self.app.estop = False
        self.app.telem_at -= 1
        self.assertTrue(self.app._compute()[0])

    def test_fault_blocks_enable(self):
        self.ready()
        self.app.telem["sf"] = 0x10
        self.app.on_press("e")
        self.assertTrue(self.app.estop)

    def test_active_steering_remains_controllable(self):
        self.ready()
        self.app.estop = False
        self.app._pressed.add("d")
        self.assertGreater(self.app._compute()[4], 0)

    def test_stale_throttle_feedback_cannot_look_like_neutral(self):
        self.ready()
        self.app.telem["tr"] = 0
        self.assertIsNone(self.app._actual_gear())

    def test_shutdown_never_commands_center_offset(self):
        self.app.steer_trim = -0.4
        self.app.shutdown()
        self.assertEqual(self.act.send_all.call_count, 5)
        for call in self.act.send_all.call_args_list:
            self.assertTrue(call.kwargs["estop"])
            self.assertEqual(call.kwargs["steer"], 0)

    def test_high_speed_ramp_is_preserved(self):
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(10, "D", 0.7), 0.30)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(11, "D", 0.7), 0.40)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(14, "D", 0.7), 0.70)

    def test_reverse_still_requires_neutral_brake_and_key_release(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app.last_motion_gear = "D"
        self.app.telem["tm"] = 1
        self.assertEqual(self.app._compute_drive(10, False, True, 0), (0, "N", 1))
        self.app.telem["tm"] = 0
        self.assertEqual(self.app._compute_drive(12, False, True, 0), (0, "N", 1))
        self.assertTrue(self.app.guard_until)
        self.app._compute_drive(12.1, False, False, 0)
        self.assertEqual(self.app.guard_until, 0)


if __name__ == "__main__":
    unittest.main()
