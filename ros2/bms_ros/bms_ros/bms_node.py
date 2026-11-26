"""ROS 2 node that polls a JBD or Daly BMS over serial.

Publishes (relative to the node namespace):
  battery_state  sensor_msgs/BatteryState    standard battery message
  status         std_msgs/String (JSON)      every decoded field, handy for web UIs / rosbridge
  /diagnostics   diagnostic_msgs/DiagnosticArray
Services:
  reconnect      std_srvs/Trigger            close and reopen the serial port
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .client import BmsClient, BmsError

TECH = {
    'unknown': BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
    'lion': BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
    'lipo': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
    'lifepo4': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
}


class BmsNode(Node):
    def __init__(self):
        super().__init__('bms')
        P = self.declare_parameter
        self.port_name = P('port', '/dev/ttyUSB0').value
        self.baud = P('baud', 9600).value
        self.protocol = P('protocol', 'jbd').value
        self.rate = P('rate', 1.0).value
        self.frame_id = P('frame_id', 'battery').value
        self.location = P('location', '').value
        self.serial_number = P('serial_number', '').value
        self.design_capacity = P('design_capacity', 0.0).value
        self.technology = P('technology', 'auto').value
        self.full_threshold = P('full_soc_threshold', 99.0).value
        self.idle_current = P('idle_current', 0.05).value
        self.timeout = P('timeout', 0.6).value
        self.debug_frames = P('debug_frames', False).value

        self.pub_batt = self.create_publisher(BatteryState, 'battery_state', qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, 'status', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_service(Trigger, 'reconnect', self._on_reconnect)

        self.serial = None
        self.client = None
        self.fails = 0
        self.timer = self.create_timer(1.0 / max(self.rate, 0.05), self._tick)
        self.get_logger().info(
            f'BMS node: {self.protocol.upper()} on {self.port_name} @ {self.baud} baud, {self.rate} Hz')

    # ------------------------------------------------------------- serial
    def _open(self):
        import serial
        try:
            self.serial = serial.Serial(self.port_name, self.baud, timeout=0.05)
        except serial.SerialException as e:
            self.serial = None
            self.get_logger().warn(f'Cannot open {self.port_name}: {e}', throttle_duration_sec=10.0)
            return False
        log = (lambda m: self.get_logger().info(m)) if self.debug_frames else None
        self.client = BmsClient(self.serial, self.protocol, timeout=self.timeout, log=log)
        self.get_logger().info(f'Opened {self.port_name}')
        return True

    def _close(self):
        try:
            if self.serial:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
        self.client = None

    def _on_reconnect(self, _req, res):
        self._close()
        res.success = self._open()
        res.message = f'{"Opened" if res.success else "Could not open"} {self.port_name}'
        return res

    # --------------------------------------------------------------- loop
    def _tick(self):
        if self.client is None and not self._open():
            self._publish_diag(None, 'Serial port not open')
            return
        try:
            s = self.client.read()
        except BmsError as e:
            self.fails += 1
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            self._publish_diag(None, str(e))
            return
        except Exception as e:  # serial unplugged etc.
            self.get_logger().error(f'Serial error: {e}; will reconnect')
            self._close()
            self._publish_diag(None, f'Serial error: {e}')
            return
        self.fails = 0
        self._publish_battery(s)
        self.pub_status.publish(String(data=json.dumps(s.to_dict())))
        self._publish_diag(s, None)

    # ---------------------------------------------------------- publishing
    def _publish_battery(self, s):
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.voltage = float(s.voltage)
        m.current = float(s.current)  # ROS convention: negative when discharging
        m.temperature = float(sum(s.temps_c) / len(s.temps_c)) if s.temps_c else math.nan
        m.charge = float(s.remaining_ah)
        m.capacity = float(s.full_ah)
        m.design_capacity = float(self.design_capacity) if self.design_capacity > 0 else math.nan
        m.percentage = float(s.soc) / 100.0 if s.soc == s.soc else math.nan
        m.present = True
        m.cell_voltage = [v / 1000.0 for v in s.cells_mv]
        m.cell_temperature = [math.nan] * len(s.cells_mv)
        m.location = self.location
        m.serial_number = self.serial_number

        if s.soc >= self.full_threshold and abs(s.current) <= self.idle_current:
            m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif s.current > self.idle_current:
            m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif s.current < -self.idle_current:
            m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        m.power_supply_health = self._health(s.faults)
        tech = self.technology
        if tech == 'auto' and s.cells_mv:
            tech = 'lifepo4' if max(s.cells_mv) < 3700 else 'lion'
        m.power_supply_technology = TECH.get(tech, BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
        self.pub_batt.publish(m)

    @staticmethod
    def _health(faults):
        if not faults:
            return BatteryState.POWER_SUPPLY_HEALTH_GOOD
        text = ' '.join(faults).lower()
        if 'over-temp' in text:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
        if 'under-temp' in text:
            return BatteryState.POWER_SUPPLY_HEALTH_COLD
        if 'over-voltage' in text:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

    def _publish_diag(self, s, error):
        st = DiagnosticStatus(name=f'{self.get_name()}: BMS', hardware_id=self.serial_number or self.port_name)
        kv = [KeyValue(key='port', value=self.port_name), KeyValue(key='protocol', value=self.protocol)]
        if s is None:
            st.level, st.message = DiagnosticStatus.ERROR, error or 'No data'
        else:
            cells = s.cells_mv
            delta = (max(cells) - min(cells)) if cells else 0
            if s.faults:
                st.level, st.message = DiagnosticStatus.ERROR, ', '.join(s.faults)
            elif s.charge_fet is False or s.discharge_fet is False or delta > 50:
                st.level = DiagnosticStatus.WARN
                st.message = 'FET off' if (s.charge_fet is False or s.discharge_fet is False) else f'Cell delta {delta} mV'
            else:
                st.level, st.message = DiagnosticStatus.OK, 'OK'
            kv += [
                KeyValue(key='voltage_V', value=f'{s.voltage:.2f}'),
                KeyValue(key='current_A', value=f'{s.current:.2f}'),
                KeyValue(key='soc_pct', value=f'{s.soc:.1f}'),
                KeyValue(key='cycles', value=str(s.cycles)),
                KeyValue(key='charge_fet', value=str(s.charge_fet)),
                KeyValue(key='discharge_fet', value=str(s.discharge_fet)),
                KeyValue(key='cell_min_mV', value=str(min(cells)) if cells else '-'),
                KeyValue(key='cell_max_mV', value=str(max(cells)) if cells else '-'),
                KeyValue(key='cell_delta_mV', value=str(delta)),
                KeyValue(key='temps_C', value=', '.join(f'{t:.1f}' for t in s.temps_c)),
            ]
        st.values = kv
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BmsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
