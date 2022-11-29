# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sim_user_interface.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from qwt.plot import QwtPlot

"""
Script généré automatiquement, 
Définit les éléments de l'interface utilisateur sur le simulateur
"""

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(799, 726)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.close_button = QtWidgets.QPushButton(self.centralwidget)
        self.close_button.setGeometry(QtCore.QRect(60, 590, 89, 25))
        self.close_button.setObjectName("close_button")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 70, 221, 182))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pause = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pause.setObjectName("pause")
        self.verticalLayout.addWidget(self.pause)
        self.reset = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.reset.setObjectName("reset")
        self.verticalLayout.addWidget(self.reset)
        self.step = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.step.setObjectName("step")
        self.verticalLayout.addWidget(self.step)
        self.circle = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.circle.setObjectName("circle")
        self.verticalLayout.addWidget(self.circle)
        self.circle_wth_tgx = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.circle_wth_tgx.setObjectName("circle_wth_tgx")
        self.verticalLayout.addWidget(self.circle_wth_tgx)
        self.POI = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.POI.setObjectName("POI")
        self.verticalLayout.addWidget(self.POI)
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(230, 10, 551, 661))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.x_graph = QwtPlot(self.verticalLayoutWidget_2)
        self.x_graph.setObjectName("x_graph")
        self.verticalLayout_2.addWidget(self.x_graph)
        self.y_graph = QwtPlot(self.verticalLayoutWidget_2)
        self.y_graph.setObjectName("y_graph")
        self.verticalLayout_2.addWidget(self.y_graph)
        self.xy_graph = QwtPlot(self.verticalLayoutWidget_2)
        self.xy_graph.setObjectName("xy_graph")
        self.verticalLayout_2.addWidget(self.xy_graph)
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(0, 320, 221, 101))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.yaw_psi = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.yaw_psi.setAlignment(QtCore.Qt.AlignCenter)
        self.yaw_psi.setObjectName("yaw_psi")
        self.verticalLayout_3.addWidget(self.yaw_psi)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.yaw_down = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.yaw_down.setObjectName("yaw_down")
        self.horizontalLayout.addWidget(self.yaw_down)
        self.yaw = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.yaw.setObjectName("yaw")
        self.horizontalLayout.addWidget(self.yaw)
        self.yaw_up = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.yaw_up.setObjectName("yaw_up")
        self.horizontalLayout.addWidget(self.yaw_up)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 799, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.close_button.setText(_translate("MainWindow", "Close"))
        self.pause.setText(_translate("MainWindow", "Pause"))
        self.reset.setText(_translate("MainWindow", "Reset"))
        self.step.setText(_translate("MainWindow", "Step"))
        self.circle.setText(_translate("MainWindow", "Circle"))
        self.circle_wth_tgx.setText(_translate("MainWindow", "Circle with tangent x axis"))
        self.POI.setText(_translate("MainWindow", "Point Of Interest"))
        self.yaw_psi.setText(_translate("MainWindow", "Yaw (psi)"))
        self.yaw_down.setText(_translate("MainWindow", "-"))
        self.yaw.setText(_translate("MainWindow", "0.0"))
        self.yaw_up.setText(_translate("MainWindow", "+"))
