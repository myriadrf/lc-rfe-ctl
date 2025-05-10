import sys
import time
import serial
import serial.tools.list_ports
from itertools import count, takewhile
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication, QMainWindow, QComboBox, QPushButton, QLabel
from PyQt6.QtGui import QPixmap

RFE_GUI_VERSION = '0.2'

# Timeout for status bar messages (ms)
STATUS_TIMEOUT_MS = 3000
# Serial response timeout in seconds for blocking commands
RESPONSE_TIMEOUT_S = 1.0
# Delay between serial commands for proper sequencing (s)
CMD_DELAY_S = 0.05

class RFEGui(QMainWindow):
    def __init__(self):
        super().__init__()

        # Load the UI file
        uic.loadUi('layout.ui', self)

        # Set window title and version
        self.setWindowTitle(f'LibreCellular RFE GUI v{RFE_GUI_VERSION}')

        self.ser = None  # Serial connection will be initialized after selecting a port

        # Get UI elements
        self.lblLogo = self.findChild(QLabel, 'lblLogo')
        self.comboPorts = self.findChild(QComboBox, 'comboPorts')
        self.btnOpenClosePort = self.findChild(QPushButton, 'btnOpenClosePort')
        self.btnRefreshPorts = self.findChild(QPushButton, 'btnRefreshPorts')
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
        self.btnRefreshPorts.clicked.connect(self.update_ports)
        self.btn5VOn.clicked.connect(lambda: self.send_command('PWR:ON:5V'))
        self.btn12VOn.clicked.connect(lambda: self.send_command('PWR:ON:12V'))
        self.btn5VOff.clicked.connect(lambda: self.send_command('PWR:OFF:5V'))
        self.btn12VOff.clicked.connect(lambda: self.send_command('PWR:OFF:12V'))
        self.btnPort1On.clicked.connect(lambda: self.send_command('RELAY:ON:1'))
        self.btnPort1Off.clicked.connect(lambda: self.send_command('RELAY:OFF:1'))
        self.btnPort2On.clicked.connect(lambda: self.send_command('RELAY:ON:2'))
        self.btnPort2Off.clicked.connect(lambda: self.send_command('RELAY:OFF:2'))
        self.btnUpdateVolts.clicked.connect(self.update_volts)
        self.btnLNA_A_Active.clicked.connect(lambda: self.send_command('LNA:ON:A'))
        self.btnLNA_A_Bypass.clicked.connect(lambda: self.send_command('LNA:OFF:A'))
        self.btnLNA_B_Active.clicked.connect(lambda: self.send_command('LNA:ON:B'))
        self.btnLNA_B_Bypass.clicked.connect(lambda: self.send_command('LNA:OFF:B'))
        self.btnPA_A_Active.clicked.connect(lambda: self.send_command('PA:ON:A'))
        self.btnPA_A_Bypass.clicked.connect(lambda: self.send_command('PA:OFF:A'))
        self.btnPA_B_Active.clicked.connect(lambda: self.send_command('PA:ON:B'))
        self.btnPA_B_Bypass.clicked.connect(lambda: self.send_command('PA:OFF:B'))
        self.btnTXInhibit_A_Active.clicked.connect(lambda: self.send_command('TDD:ON:A'))
        self.btnTXInhibit_A_Inactive.clicked.connect(lambda: self.send_command('TDD:OFF:A'))
        self.btnTXInhibit_B_Active.clicked.connect(lambda: self.send_command('TDD:ON:B'))
        self.btnTXInhibit_B_Inactive.clicked.connect(lambda: self.send_command('TDD:OFF:B'))
        self.btnPwrMeas_A_Set.clicked.connect(lambda: self.send_command(f'PWRMEAS:A:{self.comboPwrMeasMode_A.currentText()}'))
        self.btnPwrMeas_B_Set.clicked.connect(lambda: self.send_command(f'PWRMEAS:B:{self.comboPwrMeasMode_B.currentText()}'))
        self.btnPwrLevel_A_Read.clicked.connect(self.read_pwr_level_A)
        self.btnPwrLevel_B_Read.clicked.connect(self.read_pwr_level_B)
        self.btnSetAtten_A.clicked.connect(lambda: self.send_command(f'RXATTEN:A:{float(self.comboRXAtten_A.currentText()):.2f}'))
        self.btnSetAtten_B.clicked.connect(lambda: self.send_command(f'RXATTEN:B:{float(self.comboRXAtten_B.currentText()):.2f}'))
        self.btnReset_A.clicked.connect(lambda: self.send_command('RESET:A'))
        self.btnReset_B.clicked.connect(lambda: self.send_command('RESET:B'))

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
            self.statusBar().showMessage('Port Status: Closed', STATUS_TIMEOUT_MS)
        else:
            port = self.comboPorts.currentText()
            if port:
                self.ser = serial.Serial(port, 9600, timeout=5)
                self.btnOpenClosePort.setText('Close Port')
                self.send_command("VERSION")



    def send_command(self, command):
        if self.ser and self.ser.is_open:
            # flush any leftover data to align responses
            self.ser.reset_input_buffer()
            # send command and wait for device to process
            self.ser.write(f'{command}\n'.encode())
            time.sleep(CMD_DELAY_S)
            response = None
            while True:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode('utf-8').strip()
                # Print debug messages but don't use them as responses
                if line.startswith('#') or line.startswith('[') or line.startswith('>'):
                    print(line)
                    continue
                # Skip empty lines
                if not line:
                    continue
                # only accept OK, ERROR, or numeric values
                if line == 'OK' or line.startswith('ERROR:'):
                    response = line
                    break
                try:
                    float(line)
                    response = line
                    break
                except ValueError:
                    continue
            if response is None:
                return None
            if response.startswith('ERROR:'):
                reason = response.split(':',1)[1].strip()
                self.statusBar().showMessage(f'❌ Error: {reason}', STATUS_TIMEOUT_MS)
            else:
                self.statusBar().showMessage(f'✅ Command: {command}, Response: {response}', STATUS_TIMEOUT_MS)
            return response
        self.statusBar().showMessage('Port is not open.', STATUS_TIMEOUT_MS)

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

    def _update_label(self, command, label, suffix=''):
        """Send a command expecting a numeric response and update the given label."""
        val = self.send_command(command)
        if val is None or val.startswith('ERROR:'):
            self.statusBar().showMessage('❌ No response from device.', STATUS_TIMEOUT_MS)
        else:
            try:
                float(val)
            except ValueError:
                self.statusBar().showMessage('❌ Invalid response.', STATUS_TIMEOUT_MS)
            else:
                label.setText(f'{val.rstrip()}{suffix}')

    def update_volts(self):
        # update voltage labels using generic numeric handler
        self._update_label('VSENSE:5V', self.lbl5V, 'V')
        self._update_label('VSENSE:12V', self.lbl12V, 'V')
        self._update_label('VSENSE:24V', self.lbl24V, 'V')

    def read_pwr_level_A(self):
        # update power level A label
        self._update_label('PWRLEVEL:A:READ', self.lblPowerLevel_A)

    def read_pwr_level_B(self):
        # update power level B label
        self._update_label('PWRLEVEL:B:READ', self.lblPowerLevel_B)

def frange(start, stop, step):
    return takewhile(lambda x: x< stop, count(start, step))

def main():
    app = QApplication(sys.argv)
    window = RFEGui()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
