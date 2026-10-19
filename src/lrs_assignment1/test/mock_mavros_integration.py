#!/usr/bin/env python3
# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT
"""Exercise the real mission_node against a deliberately simple MAVROS mock.

This checks ROS interfaces and state transitions, NOT vehicle physics, collision
clearance, flight quality or SITL acceptance. Run in a separate ROS domain:
  source /opt/ros/jazzy/setup.bash
  ROS_DOMAIN_ID=48 python3 test/mock_mavros_integration.py --executable \
    /tmp/lrs-a123-install/lib/lrs_assignment1/mission_node --output /tmp/lrs-mock
No live vehicle/MAVROS may share that domain. The harness refuses domain 0.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '48')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String, UInt32
from std_srvs.srv import Trigger
from mavros_msgs.msg import State, ExtendedState, GPSRAW, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class MockMavros(Node):
    def __init__(self, scenario):
        super().__init__('assignment_mock_mavros')
        self.scenario = scenario
        self.position = [0., 0., 0.]
        self.velocity = [0., 0., 0.]
        self.goal = self.position.copy()
        self.feedforward = [0., 0., 0.]
        self.yaw = self.goal_yaw = 0.
        self.mode = 'STABILIZE'
        self.armed = False
        self.connected = True
        self.paused = False
        self.good_gps = scenario != 'invalid_gps'
        self.stage = 'IDLE'
        self.states = []
        self.commands = []
        self.targets = []
        self.malformed_targets = []
        self.last_tick = time.monotonic()
        self.pubs = {}
        for key, message, topic in [
            ('state', State, '/mavros/state'),
            ('extended', ExtendedState, '/mavros/extended_state'),
            ('pose', PoseStamped, '/mavros/local_position/pose'),
            ('velocity', TwistStamped, '/mavros/local_position/velocity_local'),
            ('fix', NavSatFix, '/mavros/global_position/global'),
            ('gps', GPSRAW, '/mavros/gpsstatus/gps1/raw'),
            ('satellites', UInt32, '/mavros/global_position/raw/satellites'),
            ('compass', Float64, '/mavros/global_position/compass_hdg'),
            ('altitude', Float64, '/mavros/global_position/rel_alt'),
        ]:
            self.pubs[key] = self.create_publisher(message, topic, qos_profile_sensor_data)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.target, 10)
        self.create_subscription(String, '/mission_control/state', self.state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetMode, '/mavros/set_mode', self.set_mode)
        self.create_service(CommandBool, '/mavros/cmd/arming', self.arm)
        self.create_service(CommandTOL, '/mavros/cmd/takeoff', self.takeoff)
        self.abort_client = self.create_client(Trigger, '/mission_control/abort')
        self.create_timer(.02, self.tick)

    def state(self, message):
        self.stage = message.data
        self.states.append({'time': time.monotonic(), 'state': self.stage,
                            'position': self.position.copy(), 'velocity': self.velocity.copy(), 'yaw': self.yaw})
        print(f'  {self.scenario}: {self.stage}', flush=True)

    def set_mode(self, request, response):
        self.commands.append(['mode', request.custom_mode])
        response.mode_sent = self.scenario != 'mode_rejected'
        if response.mode_sent:
            self.mode = request.custom_mode
            if self.mode == 'LAND':
                self.goal = [*self.position[:2], 0.]
                self.feedforward = [0., 0., 0.]
        return response

    def arm(self, request, response):
        self.commands.append(['arm', request.value])
        response.success = self.scenario != 'arm_rejected'
        response.result = 0 if response.success else 2
        if response.success:
            self.armed = request.value
        return response

    def takeoff(self, request, response):
        self.commands.append(['takeoff', request.altitude])
        response.success = self.scenario != 'takeoff_rejected'
        response.result = 0 if response.success else 2
        if response.success:
            self.goal = [*self.position[:2], request.altitude]
        return response

    def target(self, message):
        values = [message.position.x, message.position.y, message.position.z, message.yaw]
        if not all(math.isfinite(value) for value in values):
            self.malformed_targets.append('nonfinite')
        if message.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW):
            self.malformed_targets.append('position/yaw ignored')
        if message.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            self.malformed_targets.append('unexpected coordinate frame')
        self.targets.append({'time': time.monotonic(), 'state': self.stage,
                             'position': values[:3], 'yaw': message.yaw,
                             'actual': self.position.copy(), 'actual_yaw': self.yaw})
        if self.armed and self.mode == 'GUIDED':
            self.goal = values[:3]
            self.goal_yaw = message.yaw
            self.feedforward = [0., 0., 0.] if message.type_mask & PositionTarget.IGNORE_VX else [
                message.velocity.x, message.velocity.y, message.velocity.z]

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, .1)
        self.last_tick = now
        if self.armed:
            desired = [5*(goal-position)+ff for goal, position, ff in
                       zip(self.goal, self.position, self.feedforward)]
            speed = math.sqrt(sum(component**2 for component in desired))
            scale = min(1., 3./max(speed, 1e-9))
            self.velocity = [component*scale for component in desired]
            self.position = [position+velocity*dt for position, velocity in zip(self.position, self.velocity)]
            self.yaw += max(-6., min(6., 8*angle(self.goal_yaw-self.yaw)))*dt
            if self.mode == 'LAND' and self.position[2] < .02:
                self.position[2] = 0.
                self.armed = False  # Simulate ArduPilot auto-disarm after touchdown.
                self.velocity = [0., 0., 0.]
        else:
            self.velocity = [0., 0., 0.]
        if self.paused:
            return
        state = State()
        state.connected, state.armed, state.mode = self.connected, self.armed, self.mode
        extended = ExtendedState()
        extended.landed_state = (ExtendedState.LANDED_STATE_IN_AIR if self.position[2] > .08
                                 else ExtendedState.LANDED_STATE_ON_GROUND)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        if self.scenario == "invalid_attitude":
            pose.pose.orientation.z = pose.pose.orientation.w = 0.0
        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z = self.velocity
        fix = NavSatFix()
        fix.header = pose.header
        fix.status.status = 0 if self.good_gps else -1
        fix.latitude = 48.15+self.position[1]/111194.
        fix.longitude = 17.07+self.position[0]/74400.
        fix.altitude = 150.+self.position[2]
        gps = GPSRAW()
        gps.fix_type = 3 if self.good_gps else 1
        gps.satellites_visible = 12 if self.good_gps else 3
        satellites = UInt32(data=12 if self.good_gps else 3)
        compass = Float64(data=(90-math.degrees(self.yaw)) % 360)
        altitude = Float64(data=self.position[2])
        for key, message in [('state', state), ('extended', extended), ('pose', pose),
                             ('velocity', velocity), ('fix', fix), ('gps', gps),
                             ('satellites', satellites), ('compass', compass), ('altitude', altitude)]:
            self.pubs[key].publish(message)


def verify_mission(mock, scenario):
    states = [entry['state'] for entry in mock.states]
    require(states[-1] == 'DONE', f'Mission did not finish: {states}')
    require(not mock.armed and mock.position[2] < .05, 'DONE before touchdown/disarm')
    require(not mock.malformed_targets, f'Invalid setpoints: {mock.malformed_targets}')
    require(len(mock.targets) > 50, 'Setpoints were not streamed')
    intervals = [b['time']-a['time'] for a, b in zip(mock.targets, mock.targets[1:])
                 if a['state'] == b['state'] and a['state'] not in ['LAND', 'GROUND']]
    require(intervals and sum(intervals)/len(intervals) < .075, 'Mean publication rate below 13 Hz')
    if scenario == 'indoor':
        require(mock.commands.count(['arm', True]) == 2, 'landtakeoff must re-arm after auto-disarm')
        require(mock.commands.count(['mode', 'GUIDED']) == 2, 'landtakeoff must re-enter GUIDED')
        require(mock.commands.count(['mode', 'LAND']) == 2, 'Intermediate and final LAND missing')
        require('GROUND' in states and states.count('TAKEOFF') == 2, 'landtakeoff states missing')
        tasks = [entry for entry in mock.states if entry['state'] == 'TASK']
        require(len(tasks) == 4, 'Indoor task ordering/count differs from the mission file')
        require(math.sqrt(sum(v*v for v in tasks[0]['velocity'])) > .15,
                'Soft waypoint incorrectly required a stop')
        require(all(math.sqrt(sum(v*v for v in entry['velocity'])) < .2 for entry in tasks[1:]),
                'Hard/task waypoint advanced without settling')
        require(any(abs(angle(record['yaw']-math.pi/2)) < .01 and record['state'] == 'TASK'
                    for record in mock.targets), 'Yaw90 task was not commanded')
    else:
        require('FIGURE' in states and 'FIGURE_END' in states, 'Figure stage/completion missing')
        figure = [record for record in mock.targets if record['state'] == 'FIGURE']
        require(len(figure) > 400, 'Figure must be continuous, with a full lap of samples')
        xs = [record['position'][0] for record in figure]
        ys = [record['position'][1] for record in figure]
        require(min(xs) < -2.95 and max(xs) > 2.95 and min(ys) < -1.45 and max(ys) > 1.45,
                'Figure did not complete both lobes with the fixed 6 x 3 m geometry')
        require(all(abs(record['position'][2]-5) < 1e-6 for record in figure), 'Outdoor height is not 5 m')
        require(max(abs(b['yaw']-a['yaw']) for a, b in zip(figure, figure[1:])) < .2,
                'Figure yaw has a discontinuity')
        # Analytic derivatives and continuous heading are checked independently in core_tests.
        require(math.hypot(*mock.position[:2]) < .2, 'Outdoor mission did not return to launch')


def run_scenario(args, scenario, directory):
    mock = MockMavros(scenario)
    indoor = scenario == 'indoor'
    params = {
        'mission_type': 'indoor' if indoor else 'outdoor',
        'settle_seconds': .15, 'yaw_hold_seconds': .15, 'gps_settle_seconds': .15,
        'telemetry_timeout': .6, 'stage_timeout': 25., 'navigation_speed': 1.4, 'omega': .3,
    }
    if indoor:
        params.update(mission_file=str(directory/'mission.csv'), map_file=str(directory/'free.pcd'),
                      map_origin_x=0., map_origin_y=0., map_origin_z=0., safety_radius=.3)
    command = [str(args.executable), '--ros-args']
    for key, value in params.items():
        command += ['-p', f'{key}:={value}']
    logfile = args.output/f'{scenario}.log'
    process = None
    try:
        with logfile.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env={**os.environ, 'ROS_LOG_DIR': str(args.output/'ros')})
            begin = time.monotonic()
            triggered = False
            trigger_time = None
            count_at_abort = None
            while time.monotonic()-begin < (120 if indoor else 90):
                rclpy.spin_once(mock, timeout_sec=.02)
                elapsed = time.monotonic()-begin
                if process.poll() is not None:
                    raise AssertionError(f'mission_node exited {process.returncode}; see {logfile}')
                if scenario in ['invalid_gps', 'invalid_attitude'] and elapsed > 3:
                    require(mock.pubs['fix'].get_subscription_count() > 0, 'Controller did not discover GPS telemetry')
                    require(not mock.commands and not mock.armed, 'Controller armed without usable GPS')
                    return {'scenario': scenario, 'passed': True, 'states': mock.states, 'commands': mock.commands}
                if scenario in ['abort', 'pilot_override', 'dropout', 'gps_loss'] and mock.stage == 'NAVIGATE' and not triggered:
                    triggered, trigger_time = True, time.monotonic()
                    if scenario == 'abort':
                        require(mock.abort_client.service_is_ready(), 'Abort service missing')
                        mock.abort_client.call_async(Trigger.Request())
                    elif scenario == 'pilot_override':
                        mock.mode = 'LOITER'
                    elif scenario == 'dropout':
                        mock.paused = True
                    else:
                        mock.good_gps = False
                if mock.stage == 'ABORT':
                    require(scenario not in ['indoor', 'outdoor'], f'Unexpected abort; see {logfile}')
                    if count_at_abort is None:
                        count_at_abort = len(mock.targets)
                        trigger_time = time.monotonic()
                    if time.monotonic()-trigger_time > .6:
                        require(len(mock.targets) <= count_at_abort+1, 'Setpoints continued after ABORT')
                        if scenario in ['abort', 'gps_loss', 'takeoff_rejected']:
                            require(['mode', 'LAND'] in mock.commands, 'Abort did not request LAND')
                        if scenario == 'pilot_override':
                            require(mock.mode == 'LOITER', 'Controller overrode safety pilot mode')
                        return {'scenario': scenario, 'passed': True, 'states': mock.states, 'commands': mock.commands}
                if mock.stage == 'DONE':
                    verify_mission(mock, scenario)
                    return {'scenario': scenario, 'passed': True, 'states': mock.states,
                            'commands': mock.commands, 'target_count': len(mock.targets)}
            raise AssertionError(f'Timeout in {mock.stage}; see {logfile}')
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        mock.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('/tmp/lrs-mock'))
    parser.add_argument('--scenarios', nargs='+', default=['indoor', 'outdoor', 'invalid_gps', 'invalid_attitude', 'mode_rejected',
                                                         'arm_rejected', 'takeoff_rejected', 'abort',
                                                         'pilot_override', 'dropout', 'gps_loss'])
    args = parser.parse_args()
    require(os.environ['ROS_DOMAIN_ID'] != '0', 'Use an isolated ROS_DOMAIN_ID, not default domain 0')
    require(args.executable.is_file(), f'Executable missing: {args.executable}')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    results = []
    try:
        with tempfile.TemporaryDirectory(prefix='lrs-mock-input-') as temporary:
            directory = Path(temporary)
            (directory/'mission.csv').write_text(
                'x,y,z,precision,task\n0,0,1.2,soft,takeoff\n1,0,1.2,soft,-\n'
                '1,1,1.2,hard,landtakeoff\n1,1,1.2,hard,yaw90\n0,0,1.2,hard,land\n')
            points = [(x,y,z) for x in (-6,6) for y in (-6,6) for z in (-1,8)]
            (directory/'free.pcd').write_text(
                'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                'WIDTH 8\nHEIGHT 1\nPOINTS 8\nDATA ascii\n'+''.join(f'{x} {y} {z}\n' for x,y,z in points))
            for scenario in args.scenarios:
                print(f'MOCK scenario {scenario}', flush=True)
                try:
                    results.append(run_scenario(args, scenario, directory))
                except Exception as error:
                    results.append({'scenario': scenario, 'passed': False, 'error': str(error)})
                    print(f'FAIL: {error}', flush=True)
                (args.output/'results.json').write_text(json.dumps({'kind': 'mock, not SITL', 'results': results}, indent=2))
    finally:
        rclpy.shutdown()
    require(all(result['passed'] for result in results), f'Some mock scenarios failed: {args.output}/results.json')
    print(f'All {len(results)} MAVROS mock scenarios passed. This is not flight validation.', flush=True)


if __name__ == '__main__':
    main()
