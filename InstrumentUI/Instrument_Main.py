# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from Ui_Instrument_Main import Ui_Instrument_Main

from PyQt6.QtWidgets    import QMainWindow, QApplication
from PyQt6.QtWidgets    import QMessageBox, QFileDialog, QCompleter
from PyQt6.QtWidgets    import QInputDialog, QLineEdit
from PyQt6.QtWidgets    import QTableWidgetItem
from PyQt6.QtCore       import pyqtSlot, pyqtSignal, Qt
from PyQt6.QtGui        import QTextCursor, QColor, QFont

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from Instrument_Plot      import PlotCanvas
from Instrument_Cmd       import *
from Instrument_Thread    import *

class Instrument_Main(QMainWindow, Ui_Instrument_Main):

    # Signal
    serial_status   = pyqtSignal(bool)
    console_s       = pyqtSignal(str)

    msgbox_error_s  = pyqtSignal(str)
    msgbox_info_s   = pyqtSignal(str)

    # Variable
    serial_obj      = None                   # Instance of Serial
    serial_name     = [None, 'Refresh']      # List of serial com port
    serial_port     = None

    def __init__(self, parent=None):
        super(Instrument_Main, self).__init__(parent)
        self.setupUi(self)

        # Get UI System Parameter
        self.ui_cwd = os.getcwd()

        # Initial UI size ------------------------------------------------------------------------------------------------------
        self.setFixedSize(790, 910)

        # - Auto complete user command
        self.user_cmd_lineEdit.setCompleter(QCompleter(command_list, self.user_cmd_lineEdit))

        # Plot Canvas ----------------------------------------------------------------------------------------------------------
        self.plot_obj = PlotCanvas(self)
        self.plot_obj.move(335, 20)

        # Serial ---------------------------------------------------------------------------------------------------------------
        self.serial_obj           = DebugCommand()
        self.serial_obj.status_s  = self.serial_status
        self.serial_obj.console_s = self.console_s
        self.serial_obj.msgbox_s  = self.msgbox_error_s

        # Shared System signals ------------------------------------------------------------------------------------------------
        self.serial_status  .connect(self.ui_set_serial_status)
        self.console_s      .connect(self.ui_print)
        self.msgbox_error_s .connect(self.ui_send_error)
        self.msgbox_info_s  .connect(self.ui_send_info)

        # Threads --------------------------------------------------------------------------------------------------------------
        # - List available serial ports
        self.serial_list_port_t             = Serial_List_Port(self.serial_obj)
        self.serial_list_port_t.done_s      .connect(self.serial_list_port_done)
        self.serial_list_port_t             .start()

        # - Open serial port
        self.serial_open_port_t             = Serial_Open_Port(self.serial_obj)
        self.serial_open_port_t.done_s      .connect(self.serial_open_port_done)

        # - Serial general command
        self.serial_general_cmd_t           = Serial_General_Cmd(self.serial_obj)
        self.serial_general_cmd_t.done_s    = self.console_s

        # - Real-time Current Sensing

        # - Temperature Readout

        # Main process ---------------------------------------------------------------------------------------------------------
        # - Init

        # - Calibration

        # User interface -------------------------------------------------------------------------------------------------------

    def closeEvent(self, event):
        #if(not self.ui_send_question('Are you sure to quit ?')):
        #    event.ignore()
        #else:
        self.serial_obj.close_serial()

    # ==========================================================================================================================
    # UI - System I/O
    # ==========================================================================================================================
    # Read User input, accept --------------------------------------------------------------------------------------------------
    # - 1) Dec: '123'
    # - 2) Hex: '0xFF'
    # - 3) Bin: '0b100'
    def str2num(self, str):
        try:
            return int(str, 0)
        except:
            self.ui_send_error("Invalid Input: %s" % str)
            raise ValueError

    def str2dec(self, str):
        try:
            return float(str)
        except:
            self.ui_send_error("Invalid Input: %f" % str)
            raise ValueError

    # Set the color of ui items such as pushbutton -----------------------------------------------------------------------------
    def ui_set_color_red(self, ui_obj):
        ui_obj.setStyleSheet("background-color: #FF0000")

    def ui_set_color_green(self, ui_obj):
        ui_obj.setStyleSheet("background-color: #00FF00")

    def ui_set_color_orange(self, ui_obj):
        ui_obj.setStyleSheet("background-color: #FF8000")

    def ui_set_color_gray(self, ui_obj):
        ui_obj.setStyleSheet("background-color: #E1E1E1")

    # UI status ----------------------------------------------------------------------------------------------------------------
    # - True: Busy
    # - False: Idle
    def ui_set_serial_status(self, status):
        self.serial_status_title_label.setText('Busy' if status else 'Idle')
        self.ui_set_color_red(self.serial_status_title_label) if status else \
        self.ui_set_color_green(self.serial_status_title_label)

    def ui_get_serial_status(self):
        return False if self.serial_status_title_label.text()=='Idle' else True

    # UI message box -----------------------------------------------------------------------------------------------------------
    def ui_send_question(self, msg):
        box = QMessageBox()
        box.setWindowTitle("Question")
        box.setIcon(QMessageBox.Icon.Warning)
        box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
        box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        box.setText(msg)
        return box.exec()==QMessageBox.StandardButton.Yes

    def ui_send_error(self, msg):
        box = QMessageBox()
        box.setWindowTitle("Critical")
        box.setIcon(QMessageBox.Icon.Warning)
        box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
        box.setStandardButtons(QMessageBox.StandardButton.Yes)
        box.setText(msg)
        box.exec()

    def ui_send_warning(self, msg):
        box = QMessageBox()
        box.setWindowTitle("Warning")
        box.setIcon(QMessageBox.Icon.Warning)
        box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
        box.setStandardButtons(QMessageBox.StandardButton.Yes)
        box.setText(msg)
        box.exec()

    def ui_send_info(self, msg):
        box = QMessageBox()
        box.setWindowTitle("Information")
        box.setIcon(QMessageBox.Icon.Information)
        box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
        box.setStandardButtons(QMessageBox.StandardButton.Yes)
        box.setText(msg)
        box.exec()

    def ui_get_password(self):
        (text, ok) = QInputDialog.getText(None, "Authorization", "Password:", QLineEdit.Password)
        return text if ok else None

    # UI file dialog -----------------------------------------------------------------------------------------------------------
    def ui_select_file(self, path_init='C:/'):
        return QFileDialog.getOpenFileName(self, "Select File", path_init)

    def ui_select_folder(self, path_init='C:/'):
        return QFileDialog.getExistingDirectory(self, "Select Folder", path_init)

    # ==========================================================================================================================
    # UI - File and Folder
    # ==========================================================================================================================
    # List files and folders under the folder path -----------------------------------------------------------------------------
    def list_folder(self, folder_path):
        try:
            folder_list = os.listdir(folder_path)
        except:
            self.ui_send_error("Invalid folder path !")
            raise SystemError
        return folder_list

    # Check folder existence and create ----------------------------------------------------------------------------------------
    def create_folder_if_not_existed(self, folder_path, folder_name, ask = False):
        try:
            folder_list = self.list_folder(folder_path)
        except:
            if self.ui_send_question("Would you like to create folder %s ?" % folder_path):
                try:
                    os.mkdir(folder_path)
                    folder_list = self.list_folder(folder_path)
                except:
                    self.ui_send_error("Invalid path for folder creation'%s'" % folder_path)
                    raise IOError
            else:
                raise IOError

        if folder_name in folder_list:
            if ask:
                if not self.ui_send_question("Folder '%s' already existed !\nAre you sure to overwrite ?" % folder_name):
                    raise SystemError
        else:
            try:
                os.mkdir(folder_path + '/' + folder_name)
            except:
                self.ui_send_error("Invalid folder name: %s\nMust not contain space and \\ / : * ? \" < > |" % folder_name)
                raise IOError

    # Create experiment folder depends on the date and user information --------------------------------------------------------
    def create_experiment_folder(self, folder_path, ask=True):
        folder_name = self.ttn_get_folder_name()
        self.create_folder_if_not_existed(folder_path, folder_name, ask=ask)
        folder_path = folder_path + '/' + folder_name
        return folder_path

    # Save logs ----------------------------------------------------------------------------------------------------------------
    def save_log(self, log_path):
        try:
            time_info = get_date_time(2)                # Date and time
            log = self.console_textEdit.toPlainText()   # Log
            self.console_textEdit.clear()               # Clear

            file = open(log_path + '/' + '%s_instrument_log.txt' % time_info, 'a')
            file.write('-'*32 + '\n')
            file.write(time_info + '\n')
            file.write('-'*32 + '\n')
            file.write(log)
            file.close()

        except:
            # If the user tries to fool the UI (delete the folder during the experiment)
            # - I would not care..., currently...
            pass

    # ==========================================================================================================================
    # UI Frame - Console
    # ==========================================================================================================================
    def ui_print(self, msg):
        self.console_textEdit.moveCursor(QTextCursor.MoveOperation.End)
        self.console_textEdit.insertPlainText(msg)
        self.console_textEdit.moveCursor(QTextCursor.MoveOperation.End)

    def console_set_progress(self, value):
        self.console_progressBar.setValue(value)

    def serial_general_cmd(self, command, response=True):
        self.serial_general_cmd_t.response = response
        self.serial_general_cmd_t.command  = command
        self.serial_general_cmd_t.start()

    @pyqtSlot()
    def on_console_clear_pushButton_clicked(self):
        self.console_textEdit.clear()

    @pyqtSlot()
    def on_user_cmd_pushButton_clicked(self):
        try:
            user_cmd = self.user_cmd_lineEdit.text()
            self.user_cmd_lineEdit.clear()
            self.ui_print(">> %s\n" % user_cmd)
            self.serial_general_cmd(user_cmd)
        except:
            pass

    # ================================================================================================================================
    # UI - Update and Control
    # ================================================================================================================================
    def reset_ui(self):
        self.ui_ttn_status_update(None)

    def run_if_ready_else_exit(self, check_avail=1):
        # Check if serial is busy
        if self.ui_get_serial_status():
            self.ui_send_error('Serial is busy, you must stop first !')
            raise SystemError

    # ==========================================================================================================================
    # UI Frame - System
    # ==========================================================================================================================
    ############################################################################################################################
    # List and Open Serial Port
    ############################################################################################################################
    def serial_list_port_done(self, data):
        try:
            [name_list, port_list] = data
            self.serial_port_comboBox.clear()
            self.serial_name = name_list
            self.serial_port = port_list
            self.serial_name.append('Refresh')
            self.serial_port_comboBox.addItems(self.serial_name)
            self.serial_port_comboBox.setCurrentIndex(len(self.serial_name)-1)

            self.ui_send_info('%d valid devices found !\n' % len(port_list))
        except:
            self.ui_send_error("Error: fail to list serial port")

    def serial_open_port_done(self, data):
        try:
            if data==[]:
                self.serial_port_comboBox.setCurrentIndex(len(self.serial_name)-1)
            else:
                [ver, id, div, dac] = data
                # UI
                self.ui_send_info('Device detected ...\nFirmware:  %d\nClock: %.2f MHz' % (ver, 72.0 / (1<<div)))
        except:
            self.ui_send_error("Error: fail to open serial port")
            self.serial_port_comboBox.setCurrentIndex(len(self.serial_name)-1)

    @pyqtSlot(int)
    def on_serial_port_comboBox_activated(self, index):
        if self.serial_obj.is_connected():
            self.serial_obj.close_serial()      # Close serial
            self.reset_ui()                     # Reset ui components

        if len(self.serial_name)==index+1:
            self.serial_list_port_t.start()
        else:
            self.serial_open_port_t.config(self.serial_port[index])
            self.serial_open_port_t.start()

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setFont(QFont("Consolas", 10))
    # Enable Fusion style (better dark theme support)
    app.setStyle("Fusion")
    ui = Instrument_Main()
    ui.show()
    sys.exit(app.exec())

