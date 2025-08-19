import sys

from PySide6 import QtWidgets, QtCore, QtGui


class ToggleButton(QtWidgets.QCheckBox):
    class State:
        def __init__(self,
                     text = '',
                     button_color = QtGui.QColorConstants.White,
                     text_color = QtGui.QColorConstants.Black,
                     ):
            self.text = text
            self.button_color = button_color
            self.text_color = text_color

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setMinimumWidth(100)
        self.setMinimumHeight(50)
        self.radius = 20
        self.width = 64
        self.background_color = QtGui.QColorConstants.DarkGray
        self.checked_state = self.State(text='On',
                                        button_color=QtGui.QColorConstants.DarkBlue,
                                        text_color=QtGui.QColorConstants.White)
        self.unchecked_state = self.State(text='Off',
                                          button_color=QtGui.QColorConstants.LightGray,
                                          text_color=QtGui.QColorConstants.Black)


    def paintEvent(self, event):
        if self.isChecked():
            state = self.checked_state
        else:
            state = self.unchecked_state

        center = self.rect().center()

        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.translate(center)
        painter.setBrush(QtGui.QBrush(self.background_color))

        pen = QtGui.QPen(QtGui.QColorConstants.Black)
        pen.setWidth(2)
        painter.setPen(pen)

        painter.drawRoundedRect(
            QtCore.QRect(
                -self.width,
                -self.radius,
                2*self.width,
                2*self.radius,
            ),
            self.radius,
            self.radius,
        )

        painter.setBrush(QtGui.QBrush(state.button_color))
        sw_rect = QtCore.QRect(
            -self.radius,
            -self.radius,
            self.width + self.radius,
            2*self.radius
        )

        if not self.isChecked():
            sw_rect.moveLeft(-self.width)

        painter.drawRoundedRect(sw_rect, self.radius, self.radius)
        painter.setPen(state.text_color)
        painter.drawText(sw_rect, QtCore.Qt.AlignmentFlag.AlignCenter, state.text)
        painter.end()

    def mousePressEvent(self, event):
        super().mousePressEvent(event)

    def showEvent(self, event):
        super().showEvent(event)  # Ensure to call the super class's implementation

    def resizeEvent(self, event):
        super().resizeEvent(event)

    def hitButton(self, pos: QtCore.QPoint):
        return self.contentsRect().contains(pos)


def test():
    test_app = QtWidgets.QApplication([])
    button_widget = ToggleButton()
    button_widget.setText('Test button')
    button_widget.show()
    exit_code = test_app.exec()
    sys.exit(exit_code)


if __name__ == '__main__':
    test()

