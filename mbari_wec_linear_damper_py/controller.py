#!/usr/bin/python3

# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from buoy_api import Interface
import rclpy

import numpy as np
from scipy import interpolate


class ControlPolicy(object):
    """
    Simple Linear Damper Control Policy.

    Implements a simple linear damper controller for the piston in the WEC
    Power-Take-Off (PTO). Given motor RPM, outputs desired motor winding current (interpolated
    from RPM->Torque lookup table) to resist piston velocity. Configurable gains
    (scale/retract factor) are applied before output.
    """

    def __init__(self):
        # Define any parameter variables here
        self.Torque_constant = 0.438  # N-m/Amps
        # Desired damping Torque vs RPM relationship
        self.N_Spec = np.array([0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0])  # RPM
        self.Torque_Spec = np.array([0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6])  # N-m

        self.update_params()

    def update_params(self):
        """Update dependent variables after reading in params."""
        # Convert to Motor Winding Current vs RPM and generate interpolator for f(RPM) = I
        self.I_Spec = self.Torque_Spec / self.Torque_constant  # Amps
        self.windcurr_interp1d = interpolate.interp1d(self.N_Spec, self.I_Spec,
                                                      fill_value=self.I_Spec[-1],
                                                      bounds_error=False)

    # Modify function inputs as desired
    def target(self, rpm, scale_factor, retract_factor):
        """Calculate target value from feedback inputs."""
        N = abs(rpm)
        I = self.windcurr_interp1d(N)  # noqa: E741

        # Apply damping gain
        I *= scale_factor  # noqa: E741

        # Hysteresis due to gravity assist
        if rpm > 0.0:
            I *= -retract_factor  # noqa: E741

        return float(I)

    def __str__(self):
        return """ControlPolicy:
\tTorque_constant: {tc}
\tN_Spec: {nspec}
\tTorque_Spec: {tspec}
\tI_Spec: {ispec}""".format(tc=self.Torque_constant,
                            nspec=self.N_Spec,
                            tspec=self.Torque_Spec,
                            ispec=self.I_Spec)


class Controller(Interface):

    def __init__(self):
        super().__init__('linear_damper')

        self.policy = ControlPolicy()
        self.set_params()

        # set packet rates from controllers here
        # controller defaults to publishing feedback @ 10Hz
        # call these to set rate to 50Hz or provide argument for specific rate
        self.set_pc_pack_rate_param()  # set PC feedback publish rate to 50Hz

    # To subscribe to any topic, simply define the specific callback, e.g. power_callback
    # def power_callback(self, data):
    #     """Enables feedback of '/power_data' topic from Power Controller"""
    #     # get target value from control policy
    #     target_value = self.policy.target(data.rpm, data.scale, data.retract)

    #     # send a command, e.g. winding current
    #     self.send_pc_wind_curr_command(target_value, blocking=False)

    # Available commands to send within any callback:
    # self.send_pump_command(duration_mins, blocking=False)
    # self.send_valve_command(duration_sec, blocking=False)
    # self.send_pc_wind_curr_command(wind_curr_amps, blocking=False)
    # self.send_pc_bias_curr_command(bias_curr_amps, blocking=False)
    # self.send_pc_scale_command(scale_factor, blocking=False)
    # self.send_pc_retract_command(retract_factor, blocking=False)

    def power_callback(self, data):
        """Provide feedback of '/power_data' topic from Power Controller."""
        # Update class variables, get control policy target, send commands, etc.
        wind_curr = self.policy.target(data.rpm, data.scale, data.retract)

        self.get_logger().info('WindingCurrent:' +
                               f' f({data.rpm:.02f}, {data.scale:.02f}, {data.retract:.02f})' +
                               f' = {wind_curr:.02f}')

        self.send_pc_wind_curr_command(wind_curr, blocking=False)

    def set_params(self):
        """Use ROS2 declare_parameter and get_parameter to set policy params."""
        self.declare_parameter('torque_constant', self.policy.Torque_constant)
        self.policy.Torque_constant = \
            self.get_parameter('torque_constant').get_parameter_value().double_value

        self.declare_parameter('n_spec', self.policy.N_Spec.tolist())
        self.policy.N_Spec = \
            np.array(self.get_parameter('n_spec').get_parameter_value().double_array_value)

        self.declare_parameter('torque_spec', self.policy.Torque_Spec.tolist())
        self.policy.Torque_Spec = \
            np.array(self.get_parameter('torque_spec').get_parameter_value().double_array_value)

        # recompute any dependent variables
        self.policy.update_params()
        self.get_logger().info(str(self.policy))


def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
