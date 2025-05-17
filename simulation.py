# import asyncio
import csv
import numpy
# import qasync
# import qwt
import sim_user_interface
import sys
from uav_control_law import circle, circle_tangent_x_axis, point_of_interest, control_law

from agent_class import Agent
from PySide6 import QtCore, QtGui
from PySide6.QtGui import QPen
from PySide6.QtWidgets import QApplication
from qtm_rt.packet import RT3DMarkerPositionNoLabel

# from PyQt5 import QtCore
# from PyQt5.QtGui import QPen
# from PyQt5.QtWidgets import QApplication, QMainWindow
# from qwt import QwtPlotCurve, QwtPlotGrid
# from qtm.packet import RT3DMarkerPositionNoLabel

from flight_state_class import FlightState
from robot_class import Robot


class Window(sim_user_interface.UIMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.stopped = False
        self.psi = 0                                # (°)
        self.delta_t = 0.02                         # (s)
        self.graph_display_time = 3                 # (s)

        self.uav = Agent('cf1_sim', 'No radio')
        self.uav.position = RT3DMarkerPositionNoLabel(0.0, 0.0, 0.4, 0)
        self.uav.velocity = RT3DMarkerPositionNoLabel(0.0, 0.0, 0.0, 0)
        self.uav.yaw = 0.0
        self.robot = Robot('Cible')
        self.robot.position = RT3DMarkerPositionNoLabel(0.0, 0.0, 0.0, 0)
        self.vx_e = 0
        self.vy_e = 0
        self.x_e = 0
        self.y_e = 0
        self.previous_vx_n = 0
        self.previous_vy_n = 0
        self.previous_x_n = 0
        self.previous_y_n = 0

        file = open('sim_logs.csv', 'w')
        writer = csv.writer(file)
        writer.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                         'QTM_x (m)', 'QTM_y (m)', 'QTM_z (m)', 'cf_yaw (°)',
                         'QTM_vx (m/s)', 'QTM_vy (m/s)', 'QTM_vz (m/s)',
                         'x_g (m)', 'y_g (m)', 'z_g (m)', 'yaw_g (°)',
                         'roll_c (°)', 'pitch_c (°)', 'yaw_rate_c (°/s)', 'thrust_c (PWM)'])

        self.uav.csv_logger = writer
        self.robot.csv_logger = writer
        self.uav.state = FlightState.STANDBY
        self.graph_length = round(self.graph_display_time/self.delta_t)

        self.graph_time = [0.0] * self.graph_length
        self.graph_xm = [0.0] * self.graph_length
        self.graph_ym = [0.0] * self.graph_length
        self.graph_xg = [0.0] * self.graph_length
        self.graph_yg = [0.0] * self.graph_length
        self.graph_xr = [0.0] * self.graph_length
        self.graph_yr = [0.0] * self.graph_length
        self.graph_x_sight_left = [0.0, 0.0]
        self.graph_y_sight_left = [0.0, 0.0]
        self.graph_x_sight_right = [0.0, 0.0]
        self.graph_y_sight_right = [0.0, 0.0]

        self.xm_curve = self.x_graph.plot(self.graph_time, self.graph_xm, 'Measured x')
        self.ym_curve = self.y_graph.plot(self.graph_time, self.graph_ym, 'Measured y')

        self.xg_curve = self.x_graph.plot(self.graph_time, self.graph_xg, 'Targeted x')
        self.yg_curve = self.y_graph.plot(self.graph_time, self.graph_yg, 'Targeted y')

        self.xym_curve = self.xy_graph.plot(self.graph_xm, self.graph_ym, 'UAV position')
        self.xyr_curve = self.xy_graph.plot(self.graph_xr, self.graph_yr, 'Robot position')

        self.xy_sight_left_curve = self.xy_graph.plot(self.graph_x_sight_left, self.graph_y_sight_left)
        self.xy_sight_right_curve = self.xy_graph.plot(self.graph_x_sight_right, self.graph_y_sight_right)

        self.xm_curve.setData(self.graph_time, self.graph_xm)
        self.xm_curve.setPen(QPen(QtGui.QColorConstants.Black, 0, QtCore.Qt.PenStyle.SolidLine))
        self.ym_curve.setData(self.graph_time, self.graph_ym)
        self.ym_curve.setPen(QPen(QtGui.QColorConstants.Black, 0, QtCore.Qt.PenStyle.SolidLine))
        self.xym_curve.setData(self.graph_xm, self.graph_ym)
        self.xym_curve.setPen(QPen(QtGui.QColorConstants.Black, 0, QtCore.Qt.PenStyle.SolidLine))

        self.xg_curve.setData(self.graph_time, self.graph_xg)
        self.xg_curve.setPen(QPen(QtGui.QColorConstants.Blue, 0, QtCore.Qt.PenStyle.DotLine))
        self.yg_curve.setData(self.graph_time, self.graph_yg)
        self.yg_curve.setPen(QPen(QtGui.QColorConstants.Blue, 0, QtCore.Qt.PenStyle.DotLine))

        self.xyr_curve.setData(self.graph_xr, self.graph_yr)
        self.xyr_curve.setPen(QPen(QtGui.QColorConstants.Red, 0, QtCore.Qt.PenStyle.DashLine))

        self.xy_sight_left_curve.setData(self.graph_x_sight_left, self.graph_y_sight_left)
        self.xy_sight_left_curve.setPen(QPen(QtGui.QColorConstants.DarkGreen, 0, QtCore.Qt.PenStyle.DashDotLine))
        self.xy_sight_right_curve.setData(self.graph_x_sight_right, self.graph_y_sight_right)
        self.xy_sight_right_curve.setPen(QPen(QtGui.QColorConstants.DarkGreen, 0, QtCore.Qt.PenStyle.DashDotLine))

        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()

        # self.setup_ui(self)  # -- setup_ui was replaced by __init__ in sim_user_interface.py / UIMainWindow
        self.init_ui()
        self.show()

    def init_ui(self):
        self.close_button.clicked.connect(self.stop_button_callback)
        self.yaw_up.clicked.connect(self.yaw_up_callback)
        self.yaw_up.setEnabled(True)
        self.yaw_down.clicked.connect(self.yaw_down_callback)
        self.yaw_down.setEnabled(True)

        self.pause.clicked.connect(self.pause_button_callback)
        self.step.clicked.connect(self.step_button_callback)
        self.reset.clicked.connect(self.reset_button_callback)
        self.circle.clicked.connect(self.circle_button_callback)
        self.circle_wth_tgx.clicked.connect(self.circle_wth_tangent_x_axis_callback)
        self.POI.clicked.connect(self.point_of_interest_button_callback)

        self.x_graph.setTitle('X (m) vs time (s)')
        self.x_graph.setLabel('bottom', 'Time (s)')
        self.x_graph.setLabel('left', 'X (m)')
        self.x_graph.setXRange(-1.25, 1.25)
        self.x_graph.addLegend()

        self.y_graph.setTitle('Y (m) vs time (s)')
        self.y_graph.setLabel('bottom', 'Time (s)')
        self.y_graph.setLabel('left', 'Y (m)')
        self.y_graph.setXRange(-1.25, 1.25)
        self.y_graph.addLegend()

        self.xy_graph.setTitle('X (m) vs Y (m)')
        self.xy_graph.setLabel('bottom', 'X (m)')
        self.xy_graph.setLabel('left', 'Y (m)')
        self.xy_graph.setXRange(-1.5, 1.5)
        self.xy_graph.setYRange(-1.5, 1.5)
        self.xy_graph.showGrid(x=True, y=True)
        self.xy_graph.addLegend()

    def stop_button_callback(self):
        self.stopped = True
        self.close()

    def yaw_up_callback(self):
        self.psi = self.psi + 10        # (°)
        if self.psi > 180:
            self.psi = self.psi - 360
        self.yaw.setText(str(round(self.uav.yaw)) + ' °')

    def yaw_down_callback(self):
        self.psi = self.psi - 10        # (°)
        if self.psi < - 180:
            self.psi = self.psi + 360
        self.yaw.setText(str(round(self.uav.yaw)) + ' °')

    def pause_button_callback(self):
        self.uav.state = FlightState.NOT_FLYING

    def reset_button_callback(self):
        self.vx_e = 0
        self.vy_e = 0
        self.x_e = 0
        self.y_e = 0

        self.previous_vx_n = 0
        self.previous_vy_n = 0
        self.previous_x_n = 0
        self.previous_y_n = 0

        self.uav.position = RT3DMarkerPositionNoLabel(0.0, 0.0, 0.4, 0)
        self.uav.velocity = RT3DMarkerPositionNoLabel(0.0, 0.0, 0.0, 0)
        self.uav.yaw = 0
        self.uav.state = FlightState.STANDBY

        self.psi = 0
        self.yaw.setText(str(0) + ' °')

        self.graph_xm = [0.0] * self.graph_length
        self.graph_ym = [0.0] * self.graph_length
        self.graph_xg = [0.0] * self.graph_length
        self.graph_yg = [0.0] * self.graph_length
        self.graph_x_sight_left = [0.0, 0.0]
        self.graph_y_sight_left = [0.0, 0.0]
        self.graph_x_sight_right = [0.0, 0.0]
        self.graph_y_sight_right = [0.0, 0.0]

    def step_button_callback(self):
        self.uav.state = FlightState.STEP
        self.uav.circle_t = 0

    def circle_button_callback(self):
        self.uav.state = FlightState.CIRCLE
        self.uav.circle_t = 0

    def circle_wth_tangent_x_axis_callback(self):
        self.uav.state = FlightState.CIRCLE_TGX
        self.uav.circle_t = 0

    def point_of_interest_button_callback(self):
        self.uav.state = FlightState.POI
        self.uav.circle_t = 0

    def timer_callback(self):
        self.uav.timestamp = self.uav.timestamp + self.delta_t
        self.uav.circle_t = self.uav.circle_t + self.delta_t
        self.calculate_uav_response()
        self.update_graph()

    def calculate_uav_response(self):
        """
        Computes the behaviour of the virtual UAV
            - Gets attitude commands from the control law
            - Calculates the response of the UAV model (double-integrator) to update the position of the virtual
              UAV
        """
        if not self.uav.state == FlightState.NOT_FLYING:
            roll = 0
            pitch = 0
            yaw_rate = 0
            xg = 0
            yg = 0

            if self.uav.state == FlightState.STEP:
                xg = 1
                yg = 1
                roll, pitch, yaw_rate, _ = control_law(self.uav, xg, yg, 0.4, self.psi * numpy.pi / 180)

            if self.uav.state == FlightState.CIRCLE:
                roll, pitch, yaw_rate, _, xg, yg = circle(self.uav)

            if self.uav.state == FlightState.CIRCLE_TGX:
                roll, pitch, yaw_rate, _, xg, yg = circle_tangent_x_axis(self.uav)

            if self.uav.state == FlightState.POI:
                self.update_target_coordinates()
                roll, pitch, yaw_rate, _, xg, yg = point_of_interest(self.uav, self.robot)

            roll = roll * numpy.pi / 180
            pitch = pitch * numpy.pi / 180

            ax_n = 9.81 * pitch
            ay_n = - 9.81 * roll

            ax_e = ax_n * numpy.cos(self.uav.yaw * numpy.pi / 180) - ay_n * numpy.sin(self.uav.yaw * numpy.pi / 180)
            ay_e = ax_n * numpy.sin(self.uav.yaw * numpy.pi / 180) + ay_n * numpy.cos(self.uav.yaw * numpy.pi / 180)

            self.vx_e = self.vx_e + ax_e * self.delta_t
            self.vy_e = self.vy_e + ay_e * self.delta_t

            self.x_e = self.x_e + self.vx_e * self.delta_t
            self.y_e = self.y_e + self.vy_e * self.delta_t

            self.uav.position = RT3DMarkerPositionNoLabel(self.x_e, self.y_e, 0.4, 0)
            self.uav.velocity = RT3DMarkerPositionNoLabel(self.vx_e, self.vy_e, 0.0, 0)

            yaw = self.uav.yaw - (yaw_rate * self.delta_t)
            if yaw > 180:
                yaw = yaw - 360
            if yaw < - 180:
                yaw = yaw + 360
            self.uav.yaw = yaw
            self.yaw.setText(str(round(self.uav.yaw)) + ' °')

            self.graph_xm = self.graph_xm[1:]
            self.graph_xm.append(self.x_e)
            self.graph_ym = self.graph_ym[1:]
            self.graph_ym.append(self.y_e)

            self.graph_xr = self.graph_xr[1:]
            self.graph_xr.append(self.robot.position.x)
            self.graph_yr = self.graph_yr[1:]
            self.graph_yr.append(self.robot.position.y)

            self.graph_xg = self.graph_xg[1:]
            self.graph_xg.append(xg)
            self.graph_yg = self.graph_yg[1:]
            self.graph_yg.append(yg)

            self.graph_x_sight_left = [self.uav.position.x,
                                       self.uav.position.x + 1 * numpy.cos((self.uav.yaw + 20) * numpy.pi / 180)]
            self.graph_y_sight_left = [self.uav.position.y,
                                       self.uav.position.y + 1 * numpy.sin((self.uav.yaw + 20) * numpy.pi / 180)]
            self.graph_x_sight_right = [self.uav.position.x,
                                        self.uav.position.x + 1 * numpy.cos((self.uav.yaw - 20) * numpy.pi / 180)]
            self.graph_y_sight_right = [self.uav.position.y,
                                        self.uav.position.y + 1 * numpy.sin((self.uav.yaw - 20) * numpy.pi / 180)]

            self.graph_time = self.graph_time[1:]
            self.graph_time.append(self.uav.timestamp)

    def update_target_coordinates(self):
        radius = 0.5  # (m)
        period = 8 * numpy.pi  # (s)
        frequency = 1 / period  # (Hz)
        omega = 2 * numpy.pi * frequency  # (rad/s)
        xr = radius * numpy.cos(omega * self.uav.circle_t)
        yr = radius * numpy.sin(omega * self.uav.circle_t)

        self.robot.position = RT3DMarkerPositionNoLabel(xr, yr, 0, 0)

    def update_graph(self):
        self.xm_curve.setData(self.graph_time, self.graph_xm)
        self.ym_curve.setData(self.graph_time, self.graph_ym)
        self.xym_curve.setData(self.graph_xm, self.graph_ym)

        self.xg_curve.setData(self.graph_time, self.graph_xg)
        self.yg_curve.setData(self.graph_time, self.graph_yg)

        self.xyr_curve.setData(self.graph_xr, self.graph_yr)

        self.xy_sight_left_curve.setData(self.graph_x_sight_left, self.graph_y_sight_left)
        self.xy_sight_right_curve.setData(self.graph_x_sight_right, self.graph_y_sight_right)

        # self.x_graph.setAxisScale(2, self.graph_time[0], self.graph_time[-1])
        # self.y_graph.setAxisScale(2, self.graph_time[0], self.graph_time[-1])


def main():
    app = QApplication(sys.argv)
    w = Window()
    w.show()
    exit_code = app.exec()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
