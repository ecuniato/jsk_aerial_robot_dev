#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from .qd_nmpc_base import QDNMPCBase
from .fake_sensor import FakeSensor
from . import phys_param_beetle_omni as phys_omni


class NMPCTiltQdServoThrustDistDiffSecondOrder(QDNMPCBase):
    """
    Controller Name: Tiltable Quadrotor NMPC including Servo and Thrust Model as well as CoG Disturbance
    The controller itself is constructed in base class. The control inputs are the rates of the tilt and thrust commands.
    This file is used to define the properties of the controller, specifically, the weights and cost function for the acados solver.
    The output of the controller is the thrust and servo angle command for each rotor.
    """

    def __init__(self, build: bool = True, phys=phys_omni):
        # Model name
        self.model_name = "tilt_qd_servo_thrust_dist_differential_second_order_mdl"
        self.phys = phys

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = True  # TODO extend to include_thrust_derivative
        self.include_cog_dist_model = True
        self.include_cog_dist_parameter = (
            True  # TODO seperation between model and parameter necessary?
        )
        self.include_impedance = False
        self.differential_allocation = True
        self.actuator_second_order = True
        self.use_nullspace_goal = False

        # Read parameters from configuration file in the robot's package
        self.read_params(
            "controller",
            "nmpc",
            "beetle_omni",
            "BeetleNMPCFullServoThrustDistDiffSecondOrder.yaml",
        )

        # Create acados model & solver and generate c code
        super().__init__(build)

        # Necessary for simulation environment
        self.fake_sensor = FakeSensor(
            self.include_servo_model,
            self.include_thrust_model,
            self.include_cog_dist_model,
        )

    def get_cost_function(
        self,
        lin_acc_w=None,
        ang_acc_b=None,
        nullspace_proj=None,
        nullspace_proj_dot=None,
    ):
        # fmt: off
        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        # qe = qr^* multiply q
        q_wt_w, q_wt_x, q_wt_y, q_wt_z = self._quaternion_multiply(self.qw, self.qx, self.qy, self.qz,
                                                                   self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3])

        qe_w, qe_x, qe_y, qe_z = self._quaternion_multiply(self.qwr, -self.qxr, -self.qyr, -self.qzr,
                                                           q_wt_w, q_wt_x, q_wt_y, q_wt_z)

        # qe_x =  self.qwr * self.qx - self.qw * self.qxr - self.qyr * self.qz + self.qy * self.qzr
        # qe_y =  self.qwr * self.qy - self.qw * self.qyr + self.qxr * self.qz - self.qx * self.qzr
        # qe_z = -self.qxr * self.qy + self.qx * self.qyr + self.qwr * self.qz - self.qw * self.qzr

        rot_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)
        skew_w = self._get_skew_symmetric_matrix(self.w)

        rot_bt = self._get_rot_wb_ca(self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3])
        rot_tb = rot_bt.T

        if self.use_nullspace_goal:
            # If using nullspace goal, we want to bring our thrusts to thrust_target.
            # This is 0, but will just reduce the thrust as much as possible without compromising the main control objective, thanks to the nullspace projection.
            thrust_target = 0.0
            target_gain = 0.3
            actuators_target = -target_gain*ca.vertcat(self.ft_s - thrust_target, 0,0,0,0)  # Target actuator states (bring prop speed to 0)
            actuator_velocity_y = ca.simplify(ca.vertcat(self.ftd_s, self.ad_s) - ca.mtimes(nullspace_proj, actuators_target))
        else:
            # without nullspace goal, just minimize actuator velocity
            actuator_velocity_y = ca.vertcat(self.ftd_s, self.ad_s)

        state_y = ca.vertcat(
            self.p + rot_wb @ self.ee_p,
            self.v + rot_wb @ skew_w @ self.ee_p,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            rot_tb @ self.w,
            # self.w,
            self.a_s,
            self.ft_s,
            self.fu_b_s,
            self.tau_u_b_s,
            actuator_velocity_y[4:8], # servo angle derivatives
            actuator_velocity_y[0:4], # thrust derivatives
            self.fds_w,
            self.tau_ds_b,
        )

        state_y_e = state_y

        if self.use_nullspace_goal:
            time_contant_matrix_inv = ca.diag(ca.vertcat([1/0.0942]*4, [1/0.0480]*4)) # FIX: do not hardcode Time constant of rotor and servo
            actuators_target_jacobian = ca.jacobian(actuators_target, ca.vertcat(self.ft_s, self.a_s))
            control_y = ca.simplify(
                ca.mtimes(time_contant_matrix_inv,ca.vertcat(self.ftd_c - self.ftd_s, self.ad_c - self.ad_s))
                - ca.mtimes(nullspace_proj_dot,actuators_target)
                - ca.mtimes(ca.mtimes(nullspace_proj,actuators_target_jacobian), ca.vertcat(self.ftd_s, self.ad_s)) )
        else:
            # without nullspace goal, just minimize actuator acceleration
            control_y = ca.vertcat(
                self.ftd_c - self.ftd_s,
                self.ad_c - self.ad_s,
            )

        return state_y, state_y_e, control_y
        # fmt: on

    def get_weights(self):
        # Define Weights
        Q = np.diag(
            [
                self.params["Qp_xy"],
                self.params["Qp_xy"],
                self.params["Qp_z"],
                self.params["Qv_xy"],
                self.params["Qv_xy"],
                self.params["Qv_z"],
                0,
                self.params["Qq_xy"],
                self.params["Qq_xy"],
                self.params["Qq_z"],
                self.params["Qw_xy"],
                self.params["Qw_xy"],
                self.params["Qw_z"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qt"],
                self.params["Qt"],
                self.params["Qt"],
                self.params["Qt"],
                self.params["Qfu"],
                self.params["Qfu"],
                self.params["Qfu"],
                self.params["Qtau"],
                self.params["Qtau"],
                self.params["Qtau"],
                self.params["Qad"],
                self.params["Qad"],
                self.params["Qad"],
                self.params["Qad"],
                self.params["Qtd"],
                self.params["Qtd"],
                self.params["Qtd"],
                self.params["Qtd"],
                0,  # disturbance
                0,
                0,
                0,
                0,
                0,  # disturbance
            ]
        )
        print("Q: \n", Q)

        R = np.diag(
            [
                self.params["Rtd_c"],
                self.params["Rtd_c"],
                self.params["Rtd_c"],
                self.params["Rtd_c"],
                self.params["Rad_c"],
                self.params["Rad_c"],
                self.params["Rad_c"],
                self.params["Rad_c"],
            ]
        )
        print("R: \n", R)

        return Q, R

    def get_reference(
        self,
        target_xyz,
        target_qwxyz,
        ft_ref,
        a_ref,
        body_forces_ref,
        body_torques_ref,
        ad_ref,
        ftd_ref,
    ):
        """
        Assemble reference trajectory from target pose and reference control values.
        Gets called from reference generator class.
        Note: The definition of the reference is closely linked to the definition of the cost function.
        Therefore, this is explicitly stated in each controller file to increase comprehensiveness.

        :param target_xyz: Target position
        :param target_qwxy: Target quarternions
        :param body_forces_ref: Reference body forces
        :param body_torques_ref: Reference body torques
        :param ad_ref: Reference servo angle derivatives
        :param ftd_ref: Reference thrust derivatives
        :return xr: Reference for the state x
        :return ur: Reference for the input u
        """
        # Get dimensions
        ocp = self.get_ocp()
        nn = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]  # x
        xr[:, 1] = target_xyz[1]  # y
        xr[:, 2] = target_xyz[2]  # z
        # No reference for vx, vy, vz (idx: 3, 4, 5)
        xr[:, 6] = target_qwxyz[0]  # qx
        xr[:, 7] = target_qwxyz[1]  # qx
        xr[:, 8] = target_qwxyz[2]  # qy
        xr[:, 9] = target_qwxyz[3]  # qz
        # No reference for wx, wy, wz (idx: 10, 11, 12)
        # No reference for servo angles (idx: 13, 14, 15, 16)
        xr[:, 17] = ft_ref[0]  # f1
        xr[:, 18] = ft_ref[1]  # f2
        xr[:, 19] = ft_ref[2]  # f3
        xr[:, 20] = ft_ref[3]  # f4
        xr[:, 21] = body_forces_ref[0]
        xr[:, 22] = body_forces_ref[1]
        xr[:, 23] = body_forces_ref[2]
        xr[:, 24] = body_torques_ref[0]
        xr[:, 25] = body_torques_ref[1]
        xr[:, 26] = body_torques_ref[2]
        # No reference for disturbance (idx: 27-32)

        # Assemble input reference
        # Note: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])

        return xr, ur


if __name__ == "__main__":
    print(
        "Please run the gen_nmpc_code.py in the nmpc folder to generate the code for this controller."
    )
