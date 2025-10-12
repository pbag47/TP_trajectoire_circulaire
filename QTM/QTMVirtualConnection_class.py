
import logging
import sys

from PySide6.QtWidgets import QApplication, QPushButton

from QTM.QTMConnection_class import QTMConnection
from QTM.QTMVirtualMeasure_class import QTMVirtualMeasure


class QTMVirtualConnection(QTMConnection):
    def __init__(self, qtm_virtual_measure: QTMVirtualMeasure | None = None):
        super().__init__(ip_address="Unused IP address", parent=None)
        if qtm_virtual_measure is None:
            qtm_virtual_measure = QTMVirtualMeasure()
        self.qtm_measure = qtm_virtual_measure

    async def _connect_to_qtm(self):
        self.connection = True

    async def _disconnect_qtm(self):
        self.connection = None

    def _virtual_packet_received_callback(self, _, markers, timestamp):
        self.packet_received.emit(markers, timestamp)

    async def _start_qtm_streaming(self):
        """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
         This method is made to run forever in an asyncio event loop """
        self._logger.info('QTM virtual streaming started')
        await self.qtm_measure.stream_frames(components=['3dnolabels'], on_packet=self._virtual_packet_received_callback)

    async def _stop_qtm_streaming(self):
        await self.qtm_measure.stream_frames_stop()
        self._logger.info('QTM virtual streaming stopped')


if __name__ == '__main__':
    logging.basicConfig(filemode="QTMVirtualConnection.log")
    app = QApplication(sys.argv)
    qtm_measure = QTMVirtualMeasure([])
    qtm_connection = QTMVirtualConnection(qtm_measure)
    widget = QPushButton("Stop")
    widget.clicked.connect(qtm_connection.stop)
    widget.show()
    qtm_connection.start()
    exit_code = app.exec()
    sys.exit(exit_code)
