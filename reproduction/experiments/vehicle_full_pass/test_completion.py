"""Regression checks for the experiment's acceptance rule, with no GPU simulation."""
import unittest

from completion import PassageMonitor


def obstacles(x=0.):
    return [{'name': f'loose_{i}', 'center': [x, y, .18],
             'min_y': y - .45, 'max_y': y + .45, 'max_z': .35}
            for i, y in enumerate([-1., 0., 1.])]


def wheels(front, rear):
    return [{'name': f'wheel_{i}', 'center': [front if i < 2 else rear, -1. if i % 2 == 0 else 1., .65]}
            for i in range(4)]


class PassageAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.monitor = PassageMonitor(.47, .15, 5)
        self.monitor.update(0, -4., .47, wheels(-1., -4.), obstacles(), 0)

    def test_front_axle_alone_is_insufficient(self):
        state = self.monitor.update(1, -3., .47, wheels(1., -2.), obstacles(), 20)
        self.assertEqual(len(state['crossed_wheels']), 2)
        self.assertFalse(state['passed'])

    def test_obstacles_pushed_ahead_do_not_count_as_a_pass(self):
        self.monitor.update(1, -1., .47, wheels(1., 1.), obstacles(), 20)
        for frame in range(2, 10):
            state = self.monitor.update(frame, 2., 3.47, wheels(6., 3.), obstacles(3.), 10)
        self.assertFalse(state['passed'])
        self.assertLess(state['clearance_m'], 0.)

    def test_all_wheels_and_tail_must_remain_clear(self):
        self.monitor.update(1, -1., .47, wheels(1., 1.), obstacles(), 20)
        for frame in range(2, 6):
            state = self.monitor.update(frame, .7, .47, wheels(4., 1.), obstacles(), 0)
            self.assertFalse(state['passed'])
        state = self.monitor.update(6, .7, .47, wheels(4., 1.), obstacles(), 0)
        self.assertTrue(state['passed'])

    def test_driving_beside_obstacles_does_not_count(self):
        shifted = wheels(1., 1.)
        for wheel in shifted:
            wheel['center'][1] += 3.
        for frame in range(1, 8):
            state = self.monitor.update(frame, .7, .47, shifted, obstacles(), 10)
        self.assertFalse(state['passed'])
        self.assertEqual(state['crossed_wheels'], [])


if __name__ == '__main__':
    unittest.main()
