"""One rclpy node with its own executor thread, so API callers never deal with spinning.

All ROS callbacks run in the background executor; API methods block (with timeouts) by polling
futures and state that the executor fills in. Safe to use from a plain Python script, a Jupyter
notebook or a vision loop running in the main thread.
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class HandyNode:
    def __init__(self, name: str = 'handy_api'):
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.cb_group = ReentrantCallbackGroup()
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name=f'{name}_executor', daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # executor shut down while spinning
            pass

    @property
    def logger(self):
        return self.node.get_logger()

    def now(self):
        return self.node.get_clock().now()

    def shutdown(self):
        self._executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self._owns_context:
            rclpy.try_shutdown()


def wait_future(future, timeout: float, what: str = 'request'):
    """Block until an rclpy future completes (the executor thread completes it)."""
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            raise TimeoutError(f'{what} timed out after {timeout:.1f} s')
        time.sleep(0.005)
    return future.result()


def wait_until(predicate, timeout: float, what: str, period: float = 0.01):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > end:
            raise TimeoutError(f'timed out after {timeout:.1f} s waiting for {what}')
        time.sleep(period)
