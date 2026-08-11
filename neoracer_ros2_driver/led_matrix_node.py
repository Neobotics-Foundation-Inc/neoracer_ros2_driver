#!/usr/bin/env python3

"""
USB-UART bridge for the Neoracer's 8x8 dot-matrix display.

The panel is driven by a small firmware that renders (and auto-scrolls) ASCII
text received over a serial link. This node is a thin passthrough: it subscribes
to ``/dotmatrix/text`` (std_msgs/String) - the topic the student library's
``display.show_text()`` publishes to - and writes each message to the serial
port, appending a newline so the firmware knows the line is complete.

The panel holds the last frame written, so ``idle_text`` is written once the
port is open and again on shutdown to leave the display in a known state.

Dependencies: pyserial (``serial``).
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

import serial


class LedMatrixNode(Node):
    """Forward ``/dotmatrix/text`` strings to the USB-UART 8x8 display."""

    def __init__(self):
        super().__init__('led_matrix_node')

        # /dev/osrbot_led_matrix is the stable udev symlink to the display's
        # USB-UART adapter. baud matters here (real UART, unlike the ESP32 CDC).
        self.port = self.declare_parameter('port', '/dev/osrbot_led_matrix').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.input_topic = self.declare_parameter(
            'input_topic', '/dotmatrix/text').value
        self.append_newline = self.declare_parameter('append_newline', True).value
        # The panel's power-on frame, restated here because nothing else in the
        # stack writes it. " " for a blank idle panel.
        self.idle_text = self.declare_parameter('idle_text', 'N').value

        try:
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1,
                xonxoff=False, rtscts=False)
            self.get_logger().info(f'[INFO] LED matrix connected: {self.port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f"[ERROR] Could not open '{self.port}': {e}")
            rclpy.shutdown()
            return

        self._write_idle()

        self.create_subscription(String, self.input_topic, self._on_text, 10)
        self.get_logger().info(
            f'[INFO] LED matrix node ready, subscribed to {self.input_topic}')

    def _write(self, text: str):
        """Write one string to the display, newline-terminated."""
        if getattr(self, 'serial', None) is None or not self.serial.is_open:
            return
        if self.append_newline and (not text or text[-1] != '\n'):
            text += '\n'
        try:
            self.serial.write(text.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'[ERROR] Could not write to display: {e}')

    def _write_idle(self):
        """Write the idle frame, unless ``idle_text`` resolved to no value."""
        if self.idle_text is not None:
            self._write(self.idle_text)

    def _on_text(self, msg: String):
        """Write an incoming string to the display."""
        self._write(msg.data)

    def destroy_node(self):
        """Restore the idle frame, then close the serial port."""
        if getattr(self, 'serial', None) is not None and self.serial.is_open:
            self._write_idle()
            self.serial.close()
        super().destroy_node()


def main(args=None):
    """Spin the LED matrix node until shutdown."""
    # ALL puts SIGTERM on rclpy's handler so the teardown below runs under
    # systemd; a signal.signal() handler would not run until rcl_wait returned.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.ALL)
    node = LedMatrixNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
