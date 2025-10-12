
import asyncio
import logging
import qtm_rt
import threading
import time
import sys

from PySide6.QtCore import Slot
from PySide6 import QtWidgets
from queue import SimpleQueue
from _queue import Empty
from typing import Callable


class QTMConnection:
    def __init__(self,
                 ip_address: str = '192.168.0.1',
                 on_packet: Callable | None = None,
                 ):
        self._logger = logging.getLogger(self.__class__.__name__)
        self.ip_address: str = ip_address
        self.on_packet: Callable | None = on_packet
        self.connection: qtm_rt.QRTConnection | bool | None = None
        self.busy = threading.Event()
        self.stop_flag = threading.Event()
        self.stack = SimpleQueue()
        self.thread = threading.Thread(
            target=self._main,
            daemon=True,
            name='QTM connection thread',
        )
        self.thread.start()

    def add_task(self, method: Callable, *args, **kwargs):
        self.stack.put_nowait((method, args, kwargs))

    def start(self):
        self.add_task(self._connect_to_qtm)
        self.add_task(self._start_qtm_streaming)

    def connect_qtm(self):
        self.add_task(self._connect_to_qtm)

    @Slot()
    def stop(self):
        if self.connection is None:
            self.stop_flag.set()
            self._logger.error("No QTM connection to disconnect from")
            return
        self.add_task(self._stop_qtm_streaming)
        self.add_task(self._disconnect_qtm)
        self.stop_flag.set()

    def _packet_received_callback(self, packet: qtm_rt.QRTPacket):
        timestamp = packet.timestamp * 10**-6
        headers, markers = packet.get_3d_markers_no_label()
        for marker in markers:
            self.convert_marker_units(marker)
        if self.on_packet:
            self.on_packet(markers, timestamp)

    @staticmethod
    def convert_marker_units(marker: qtm_rt.packet.RT3DMarkerPositionNoLabel):
        """
        Marker instances come with x, y and z coordinates in mm.
        This method converts coordinates units to m
        """
        marker.x = marker.x * 10**-3
        marker.y = marker.y * 10**-3
        marker.z = marker.z * 10**-3

    async def _connect_to_qtm(self):
        try:
            self.connection = await asyncio.wait_for(qtm_rt.connect(self.ip_address), timeout=5)
        except TimeoutError:
            self._logger.error('Timeout: no response')
            self.connection = None
        if self.connection is None:
            self._logger.error('Error during QTM connection @ ' + self.ip_address)
            return
        self._logger.info('QTM connected @ ' + self.ip_address)

    async def _start_qtm_streaming(self):
        """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
         This method is made to run forever in an asyncio event loop """
        self._logger.info('QTM stream started')
        await self.connection.stream_frames(components=['3dnolabels'], on_packet=self._packet_received_callback)

    async def _stop_qtm_streaming(self):
        await self.connection.stream_frames_stop()
        self._logger.info('QTM stream stopped')

    async def _disconnect_qtm(self):
        if self.connection is None:
            self._logger.warning('Attempted to close a non-existing QTM connection')
            return
        await self.connection.stream_frames_stop()
        self.connection.disconnect()
        self._logger.info('QTM disconnected')

    def _main(self):
        self._logger.info(self.thread.name + ' started')
        while (not self.stop_flag.is_set()) or self.busy.is_set():
            time.sleep(0.25)
            try:
                method, args, kwargs = self.stack.get_nowait()
            except Empty:
                self.busy.clear()
                continue
            self.busy.set()
            self._logger.info("Run: " + method.__name__)
            asyncio.run(method(*args, **kwargs))
            self._logger.info("Done: " + method.__name__)
        self._logger.info(self.thread.name + " stopped")


def test():
    logging.basicConfig(filemode="QTMConnection.log")
    app = QtWidgets.QApplication(sys.argv)
    qtm_connection = QTMConnection()
    widget = QtWidgets.QFrame()
    layout = QtWidgets.QVBoxLayout()
    connect_button = QtWidgets.QPushButton('Connect to QTM')
    start_button = QtWidgets.QPushButton('Start')
    stop_button = QtWidgets.QPushButton("Stop")
    connect_button.clicked.connect(qtm_connection.connect_qtm)
    start_button.clicked.connect(qtm_connection.start)
    stop_button.clicked.connect(qtm_connection.stop)
    layout.addWidget(connect_button)
    layout.addWidget(start_button)
    layout.addWidget(stop_button)
    widget.setLayout(layout)
    widget.show()
    exit_code = app.exec()
    sys.exit(exit_code)


if __name__ == '__main__':
    test()
