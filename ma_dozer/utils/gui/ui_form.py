# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'C:\Users\Yuval\Documents\dozer.ui'
#
# Created by: PyQt5 UI code generator 5.14.2
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(269, 127)
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setGeometry(QtCore.QRect(10, 10, 241, 111))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.Left = QtWidgets.QPushButton(self.widget)
        self.Left.setObjectName("Left")
        self.horizontalLayout.addWidget(self.Left)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.Forward = QtWidgets.QPushButton(self.widget)
        self.Forward.setObjectName("Forward")
        self.verticalLayout.addWidget(self.Forward)
        self.Stop = QtWidgets.QPushButton(self.widget)
        self.Stop.setObjectName("Stop")
        self.verticalLayout.addWidget(self.Stop)
        self.Backward = QtWidgets.QPushButton(self.widget)
        self.Backward.setObjectName("Backward")
        self.verticalLayout.addWidget(self.Backward)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.Right = QtWidgets.QPushButton(self.widget)
        self.Right.setObjectName("Right")
        self.horizontalLayout.addWidget(self.Right)

        self.retranslateUi(Form)
        self.Left.clicked.connect(Form.left_clicked)
        self.Backward.clicked.connect(Form.backward_clicked)
        self.Right.clicked.connect(Form.right_clicked)
        self.Stop.clicked.connect(Form.stop_clicked)
        self.Forward.clicked.connect(Form.forward_clicked)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.Left.setText(_translate("Form", "Left"))
        self.Forward.setText(_translate("Form", "Forward"))
        self.Stop.setText(_translate("Form", "Stop"))
        self.Backward.setText(_translate("Form", "Backward"))
        self.Right.setText(_translate("Form", "Right"))
