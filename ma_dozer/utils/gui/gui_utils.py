from PyQt5.QtWidgets import (QMainWindow)
from typing import Union

from ma_dozer.scripts.dozer.dozer_controller_manager import DozerControlManager
from ma_dozer.scripts.dumper.dumper_controller_manager import DumperControlManager
from ma_dozer.utils.gui.ui_form import Ui_Form


class Window(QMainWindow, Ui_Form):

    def __init__(self, controller_manager: Union[DozerControlManager, DumperControlManager], parent=None, ):
        super().__init__(parent)
        self.setupUi(self)
        self.controller_manager = controller_manager

    def left_clicked(self):
        self.controller_manager.controller.roboclaw_controller.rotate_left()

    def right_clicked(self):
        self.controller_manager.controller.roboclaw_controller.rotate_right()

    def stop_clicked(self):
        self.controller_manager.stop()

    def forward_clicked(self):
        self.controller_manager.controller.roboclaw_controller.forward()

    def backward_clicked(self):
        self.controller_manager.controller.roboclaw_controller.backward()
