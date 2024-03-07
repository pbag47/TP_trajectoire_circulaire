import asyncio
import csv
import numpy
import qasync
import qwt
import sim_user_interface
import sys
import uav_control_law

from agent_class import Agent
from PyQt5 import QtCore
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QApplication, QMainWindow
from qwt import QwtPlotCurve, QwtPlotGrid
from qtm.packet import RT3DMarkerPositionNoLabel

from flight_state_class import FlightState
from robot_class import Robot


class Window(QMainWindow, sim_user_interface.Ui_MainWindow):
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

        self.xm_curve = QwtPlotCurve('Measured x')
        self.ym_curve = QwtPlotCurve('Measured y')

        self.xg_curve = QwtPlotCurve('Targeted x')
        self.yg_curve = QwtPlotCurve('Targeted y')

        self.xym_curve = QwtPlotCurve('UAV position')
        self.xyr_curve = QwtPlotCurve('Robot position')

        self.xy_sight_left_curve = QwtPlotCurve('UAV sight left limit')
        self.xy_sight_right_curve = QwtPlotCurve('UAV sight right limit')

        self.xm_curve.setData(self.graph_time, self.graph_xm)
        self.xm_curve.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.SolidLine))
        self.ym_curve.setData(self.graph_time, self.graph_ym)
        self.ym_curve.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.SolidLine))
        self.xym_curve.setData(self.graph_xm, self.graph_ym)
        self.xym_curve.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.SolidLine))

        self.xg_curve.setData(self.graph_time, self.graph_xg)
        self.xg_curve.setPen(QPen(QtCore.Qt.blue, 0, QtCore.Qt.DotLine))
        self.yg_curve.setData(self.graph_time, self.graph_yg)
        self.yg_curve.setPen(QPen(QtCore.Qt.blue, 0, QtCore.Qt.DotLine))

        self.xyr_curve.setData(self.graph_xr, self.graph_yr)
        self.xyr_curve.setPen(QPen(QtCore.Qt.red, 0, QtCore.Qt.DashLine))

        self.xy_sight_left_curve.setData(self.graph_x_sight_left, self.graph_y_sight_left)
        self.xy_sight_left_curve.setPen(QPen(QtCore.Qt.darkGreen, 0, QtCore.Qt.DashDotLine))
        self.xy_sight_right_curve.setData(self.graph_x_sight_right, self.graph_y_sight_right)
        self.xy_sight_right_curve.setPen(QPen(QtCore.Qt.darkGreen, 0, QtCore.Qt.DashDotLine))

        self.setupUi(self)
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
        self.x_graph.setAxisTitle(2, 'Time (s)')
        self.x_graph.setAxisTitle(0, 'X (m)')
        self.x_graph.setAxisScale(0, -1.25, 1.25)

        self.y_graph.setTitle('Y (m) vs time (s)')
        self.y_graph.setAxisTitle(2, 'Time (s)')
        self.y_graph.setAxisTitle(0, 'Y (m)')
        self.y_graph.setAxisScale(0, -1.25, 1.25)

        self.xy_graph.setTitle('X (m) vs Y (m)')
        self.xy_graph.setAxisTitle(2, 'X (m)')
        self.xy_graph.setAxisTitle(0, 'Y (m)')
        self.xy_graph.setAxisScale(0, -1.5, 1.5)
        self.xy_graph.setAxisScale(2, -1.5, 1.5)

        xy_graph_grid = QwtPlotGrid()
        xy_graph_grid.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.DotLine))

        self.xm_curve.attach(self.x_graph)
        self.xg_curve.attach(self.x_graph)

        self.ym_curve.attach(self.y_graph)
        self.yg_curve.attach(self.y_graph)

        self.xym_curve.attach(self.xy_graph)
        self.xyr_curve.attach(self.xy_graph)
        self.xy_sight_left_curve.attach(self.xy_graph)
        self.xy_sight_right_curve.attach(self.xy_graph)
        xy_graph_grid.attach(self.xy_graph)

        self.x_graph.insertLegend(qwt.QwtLegend(), qwt.QwtPlot.RightLegend)
        self.y_graph.insertLegend(qwt.QwtLegend(), qwt.QwtPlot.RightLegend)
        self.xy_graph.insertLegend(qwt.QwtLegend(), qwt.QwtPlot.RightLegend)

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

    async def timer(self):
        while not self.stopped:
            await asyncio.sleep(self.delta_t)
            self.uav.timestamp = self.uav.timestamp + self.delta_t
            self.uav.circle_t = self.uav.circle_t + self.delta_t
            self.calculate_uav_response()

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
                roll, pitch, yaw_rate, _ = uav_control_law.control_law(self.uav, xg, yg, 0.4, self.psi * numpy.pi / 180)

            if self.uav.state == FlightState.CIRCLE:
                roll, pitch, yaw_rate, _, xg, yg = uav_control_law.circle(self.uav)

            if self.uav.state == FlightState.CIRCLE_TGX:
                roll, pitch, yaw_rate, _, xg, yg = uav_control_law.circle_tangent_x_axis(self.uav)

            if self.uav.state == FlightState.POI:
                self.update_target_coordinates()
                roll, pitch, yaw_rate, _, xg, yg = uav_control_law.point_of_interest(self.uav, self.robot)

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

    async def update_graph(self):
        while not self.stopped:
            await asyncio.sleep(self.delta_t)
            self.xm_curve.setData(self.graph_time, self.graph_xm)
            self.ym_curve.setData(self.graph_time, self.graph_ym)
            self.xym_curve.setData(self.graph_xm, self.graph_ym)

            self.xg_curve.setData(self.graph_time, self.graph_xg)
            self.yg_curve.setData(self.graph_time, self.graph_yg)

            self.xyr_curve.setData(self.graph_xr, self.graph_yr)

            self.xy_sight_left_curve.setData(self.graph_x_sight_left, self.graph_y_sight_left)
            self.xy_sight_right_curve.setData(self.graph_x_sight_right, self.graph_y_sight_right)

            self.x_graph.setAxisScale(2, self.graph_time[0], self.graph_time[-1])
            self.y_graph.setAxisScale(2, self.graph_time[0], self.graph_time[-1])

            self.x_graph.replot()
            self.y_graph.replot()
            self.xy_graph.replot()


def main():
    app = QApplication(sys.argv)
    w = Window()
    q_loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(q_loop)
    asyncio.ensure_future(w.update_graph())
    asyncio.ensure_future(w.timer())
    asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    main()
