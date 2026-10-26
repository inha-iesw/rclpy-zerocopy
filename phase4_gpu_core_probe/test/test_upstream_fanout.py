"""Exercise unchanged upstream C++ processes and observe scalar GPU results."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


def test_upstream_cpp_gpu_fanout(tmp_path):
    """Both independent GPU consumers process frames through the native core."""
    prefix = Path(get_package_prefix('multi_process_image_fanout'))
    binaries = prefix / 'lib' / 'multi_process_image_fanout'
    topic = f'/g3_probe_{os.getpid()}'
    received = {'encoder': [], 'inference': []}
    context = rclpy.context.Context()
    node = None
    executor = None
    processes = []
    logs = []
    try:
        rclpy.init(context=context)
        node = rclpy.create_node(f'g3_observer_{os.getpid()}', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        for role in received:
            node.create_subscription(
                String, f'{topic}/{role}',
                lambda message, role=role: received[role].append(json.loads(message.data)), 10)
        commands = [
            ('encoder_like_node', ['input_topic_name:=' + topic,
                                   'status_topic_name:=' + topic + '/encoder']),
            ('inference_like_node', ['input_topic_name:=' + topic,
                                     'status_topic_name:=' + topic + '/inference']),
            ('gpu_image_publisher', ['topic_name:=' + topic, 'width:=64', 'height:=48',
                                     'block_count:=4', 'publish_rate_hz:=20.0']),
        ]
        for executable, parameters in commands:
            log = (tmp_path / (executable + '.log')).open('w')
            logs.append(log)
            command = [str(binaries / executable), '--ros-args']
            for parameter in parameters:
                command += ['-p', parameter]
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and min(map(len, received.values())) < 10:
            assert all(p.poll() is None for p in processes), 'upstream process exited'
            executor.spin_once(timeout_sec=0.1)
        assert min(map(len, received.values())) >= 10, received
        for role, statuses in received.items():
            assert all(s['checksum'] > 0 and s['stamp_ns'] > 0 for s in statuses), role
            assert statuses[-1]['received'] >= 10, role
        assert received['encoder'][-1]['output_width'] == 32
        assert received['encoder'][-1]['output_height'] == 24
        assert all(0 <= s['min'] <= s['mean'] <= s['max'] <= 1
                   for s in received['inference'])
        (tmp_path / 'observations.json').write_text(json.dumps(received, indent=2))
    finally:
        # Stop producers first; terminate only the subprocesses this test owns.
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
