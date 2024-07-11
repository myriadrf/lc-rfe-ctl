import sys
import time
import serial
import serial.tools.list_ports
from itertools import count, takewhile
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication, QMainWindow, QComboBox, QPushButton, QLabel
from PyQt6.QtGui import QPixmap

class RFEGui(QMainWindow):
    def __init__(self):
        super().__init__()

        # Load the UI file
        uic.loadUi('layout.ui', self)

        self.ser = None  # Serial connection will be initialized after selecting a port

        # Get UI elements
        self.lblLogo = self.findChild(QLabel, 'lblLogo')
        self.comboPorts = self.findChild(QComboBox, 'comboPorts')
        self.btnOpenClosePort = self.findChild(QPushButton, 'btnOpenClosePort')
        self.btn5VOn = self.findChild(QPushButton, 'btn5VOn')
        self.btn12VOn = self.findChild(QPushButton, 'btn12VOn')
        self.btn5VOff = self.findChild(QPushButton, 'btn5VOff')
        self.btn12VOff = self.findChild(QPushButton, 'btn12VOff')
        self.btnPort1On = self.findChild(QPushButton, 'btnPort1On')
        self.btnPort1Off = self.findChild(QPushButton, 'btnPort1Off')
        self.btnPort2On = self.findChild(QPushButton, 'btnPort2On')
        self.btnPort2Off = self.findChild(QPushButton, 'btnPort2Off')
        self.btnUpdateVolts = self.findChild(QPushButton, 'btnUpdateVolts')
        self.lbl5V = self.findChild(QLabel, 'lbl5V')
        self.lbl12V = self.findChild(QLabel, 'lbl12V')
        self.lbl24V = self.findChild(QLabel, 'lbl24V')
        self.btnLNA_A_Active = self.findChild(QPushButton, 'btnLNA_A_Active')
        self.btnLNA_A_Bypass = self.findChild(QPushButton, 'btnLNA_A_Bypass')
        self.btnLNA_B_Active = self.findChild(QPushButton, 'btnLNA_B_Active')
        self.btnLNA_B_Bypass = self.findChild(QPushButton, 'btnLNA_B_Bypass')
        self.btnPA_A_Active = self.findChild(QPushButton, 'btnPA_A_Active')
        self.btnPA_A_Bypass = self.findChild(QPushButton, 'btnPA_A_Bypass')
        self.btnPA_B_Active = self.findChild(QPushButton, 'btnPA_B_Active')
        self.btnPA_B_Bypass = self.findChild(QPushButton, 'btnPA_B_Bypass')
        self.btnTXInhibit_A_Active = self.findChild(QPushButton, 'btnTXInhibit_A_Active')
        self.btnTXInhibit_A_Inactive = self.findChild(QPushButton, 'btnTXInhibit_A_Inactive')
        self.btnTXInhibit_B_Active = self.findChild(QPushButton, 'btnTXInhibit_B_Active')
        self.btnTXInhibit_B_Inactive = self.findChild(QPushButton, 'btnTXInhibit_B_Inactive')
        self.comboPwrMeasMode_A = self.findChild(QComboBox, 'comboPwrMeasMode_A')
        self.comboPwrMeasMode_B = self.findChild(QComboBox, 'comboPwrMeasMode_B')
        self.btnPwrMeas_A_Set = self.findChild(QPushButton, 'btnPwrMeas_A_Set')
        self.btnPwrMeas_B_Set = self.findChild(QPushButton, 'btnPwrMeas_B_Set')
        self.lblPowerLevel_A = self.findChild(QLabel, 'lblPowerLevel_A')
        self.lblPowerLevel_B = self.findChild(QLabel, 'lblPowerLevel_B')
        self.btnPwrLevel_A_Read = self.findChild(QPushButton, 'btnPwrLevel_A_Read')
        self.btnPwrLevel_B_Read = self.findChild(QPushButton, 'btnPwrLevel_B_Read')
        self.comboRXAtten_A = self.findChild(QComboBox, 'comboRXAtten_A')
        self.comboRXAtten_B = self.findChild(QComboBox, 'comboRXAtten_B')
        self.btnSetAtten_A = self.findChild(QPushButton, 'btnSetAtten_A')
        self.btnSetAtten_B = self.findChild(QPushButton, 'btnSetAtten_B')
        self.btnReset_A = self.findChild(QPushButton, 'btnReset_A')
        self.btnReset_B = self.findChild(QPushButton, 'btnReset_B')

        # Connect buttons to functions
        self.btnOpenClosePort.clicked.connect(self.toggle_port)
        self.btn5VOn.clicked.connect(lambda: self.send_command('5V_ON'))
        self.btn12VOn.clicked.connect(lambda: self.send_command('12V_ON'))
        self.btn5VOff.clicked.connect(lambda: self.send_command('5V_OFF'))
        self.btn12VOff.clicked.connect(lambda: self.send_command('12V_OFF'))
        self.btnPort1On.clicked.connect(lambda: self.send_command('RELAY1_ON'))
        self.btnPort1Off.clicked.connect(lambda: self.send_command('RELAY1_OFF'))
        self.btnPort2On.clicked.connect(lambda: self.send_command('RELAY2_ON'))
        self.btnPort2Off.clicked.connect(lambda: self.send_command('RELAY2_OFF'))
        self.btnUpdateVolts.clicked.connect(self.update_volts)
        self.btnLNA_A_Active.clicked.connect(lambda: self.send_command('LNA_A_ACTIVE'))
        self.btnLNA_A_Bypass.clicked.connect(lambda: self.send_command('LNA_A_BYPASS'))
        self.btnLNA_B_Active.clicked.connect(lambda: self.send_command('LNA_B_ACTIVE'))
        self.btnLNA_B_Bypass.clicked.connect(lambda: self.send_command('LNA_B_BYPASS'))
        self.btnPA_A_Active.clicked.connect(lambda: self.send_command('PA_A_ACTIVE'))
        self.btnPA_A_Bypass.clicked.connect(lambda: self.send_command('PA_A_BYPASS'))
        self.btnPA_B_Active.clicked.connect(lambda: self.send_command('PA_B_ACTIVE'))
        self.btnPA_B_Bypass.clicked.connect(lambda: self.send_command('PA_B_BYPASS'))
        self.btnTXInhibit_A_Active.clicked.connect(lambda: self.send_command('TXINHIBIT_A_ACTIVE'))
        self.btnTXInhibit_A_Inactive.clicked.connect(lambda: self.send_command('TXINHIBIT_A_INACTIVE'))
        self.btnTXInhibit_B_Active.clicked.connect(lambda: self.send_command('TXINHIBIT_B_ACTIVE'))
        self.btnTXInhibit_B_Inactive.clicked.connect(lambda: self.send_command('TXINHIBIT_B_INACTIVE'))
        self.btnTXRXLoop_A_Active.clicked.connect(lambda: self.send_command('TXRXLOOP_A_ACTIVE'))
        self.btnTXRXLoop_A_Inactive.clicked.connect(lambda: self.send_command('TXRXLOOP_A_INACTIVE'))
        self.btnTXRXLoop_B_Active.clicked.connect(lambda: self.send_command('TXRXLOOP_B_ACTIVE'))
        self.btnTXRXLoop_B_Inactive.clicked.connect(lambda: self.send_command('TXRXLOOP_B_INACTIVE'))
        self.btnPwrMeas_A_Set.clicked.connect(lambda: self.send_command(f'PWRMEAS_A_{self.comboPwrMeasMode_A.currentText()}'))
        self.btnPwrMeas_B_Set.clicked.connect(lambda: self.send_command(f'PWRMEAS_B_{self.comboPwrMeasMode_B.currentText()}'))
        self.btnPwrLevel_A_Read.clicked.connect(lambda: self.lblPowerLevel_A.setText(self.send_command('PWRLEVEL_A_READ').rstrip()))
        self.btnPwrLevel_B_Read.clicked.connect(lambda: self.lblPowerLevel_B.setText(self.send_command('PWRLEVEL_B_READ').rstrip()))
        self.btnSetAtten_A.clicked.connect(lambda: self.send_command(f'RXATTEN_A_{float(self.comboRXAtten_A.currentText()):.2f}'))
        self.btnSetAtten_B.clicked.connect(lambda: self.send_command(f'RXATTEN_B_{float(self.comboRXAtten_B.currentText()):.2f}'))
        self.btnReset_A.clicked.connect(lambda: self.send_command('RESET_A'))
        self.btnReset_B.clicked.connect(lambda: self.send_command('RESET_B'))

        # Load logo image
        logoPixmap = QPixmap('lc_logo.png')
        self.lblLogo.setPixmap(logoPixmap.scaled(100, 100))

        # Update port list
        self.update_ports()

        # Populate combo boxes
        self.comboPwrMeasMode_A.addItems(['OFF', 'SDR', 'EXT'])
        self.comboPwrMeasMode_B.addItems(['OFF', 'SDR', 'EXT'])

        for i in frange(0, 32, 0.25):
            self.comboRXAtten_A.addItem(str(i))
            self.comboRXAtten_B.addItem(str(i))


    def update_ports(self):
        ports = serial.tools.list_ports.comports()
        self.comboPorts.clear()
        self.comboPorts.addItems([port.device for port in ports])

    def toggle_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.btnOpenClosePort.setText('Open Port')
            self.statusBar().showMessage('Port Status: Closed')
        else:
            port = self.comboPorts.currentText()
            if port:
                self.ser = serial.Serial(port, 9600, timeout=1)
                self.btnOpenClosePort.setText('Close Port')

                self.ser.write("VERSION\n".encode())
                time.sleep(0.1)
                self.ser.reset_input_buffer()

                self.update_volts()

                self.ser.write("VERSION\n".encode())
                self.statusBar().showMessage(f'Port Status: Open ({port}), FW Version: {self.ser.readline().decode("utf-8").rstrip()}')

    def send_command(self, command):
        if self.ser and self.ser.is_open:
            command_term = f'{command}\n'
            self.ser.write(command_term.encode())  # Send the command
            response = self.ser.readline().decode('utf-8')  # Read the response
            self.statusBar().showMessage(f'Command: {command}, Response: {response}')  # Append the response to the text box
            return response
        else:
            self.statusBar().showMessage('Port is not open.')

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

    def update_volts(self):
        val5V = self.send_command('VSENSE_5V')
        val12V = self.send_command('VSENSE_12V')
        val24V = self.send_command('VSENSE_24V')

        self.lbl5V.setText(f'{val5V.rstrip()}V')
        self.lbl12V.setText(f'{val12V.rstrip()}V')
        self.lbl24V.setText(f'{val24V.rstrip()}V')


def frange(start, stop, step):
    return takewhile(lambda x: x< stop, count(start, step))

def main():
    app = QApplication(sys.argv)
    window = RFEGui()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
