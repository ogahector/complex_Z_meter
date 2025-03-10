# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from Ui_Instrument_Main import Ui_InstrumentMain

from PyQt6.QtWidgets    import QMainWindow, QApplication
from PyQt6.QtWidgets    import QFileDialog, QCompleter
from PyQt6.QtWidgets    import QTableWidgetItem
from PyQt6.QtCore       import pyqtSlot, pyqtSignal, Qt
from PyQt6.QtGui        import QTextCursor, QColor, QFont

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from Instrument_Plot      import PlotCanvas
# from Instrument_Cmd       import *
from TEST_Instrument_Cmd    import *
from Instrument_Thread    import *
from Instrument_Func      import *

class InstrumentMain(QMainWindow, Ui_InstrumentMain):

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
        super(InstrumentMain, self).__init__(parent)
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
        self.serial_obj           = TESTInstrumentCmd()
        # self.serial_obj           = DebugCommand()
        self.serial_obj.status_s  = self.serial_status
        self.serial_obj.console_s = self.console_s
        self.serial_obj.msgbox_s  = self.msgbox_error_s

        # Shared System signals ------------------------------------------------------------------------------------------------
        self.serial_status  .connect(self.ui_set_serial_status)
        self.console_s      .connect(self.ui_print)
        self.msgbox_error_s .connect(ui_send_error)
        self.msgbox_info_s  .connect(ui_send_info)

        # Threads --------------------------------------------------------------------------------------------------------------
        # - List available serial ports
        self.serial_list_port_t             = SerialListPort(self.serial_obj)
        self.serial_list_port_t.done_s      .connect(self.serial_list_port_done)
        self.serial_list_port_t             .start()

        # - Open serial port
        self.serial_open_port_t             = SerialOpenPort(self.serial_obj)
        self.serial_open_port_t.done_s      .connect(self.serial_open_port_done)

        # - Serial general command
        self.serial_general_cmd_t           = SerialGeneralCmd(self.serial_obj)
        self.serial_general_cmd_t.done_s    = self.console_s

        # - Real-time Current Sensing
        self.current_read_t                 = CurrentRead(self.serial_obj)
        self.current_read_t.done_s          = self.console_s

        # - Temperature Readout
        self.adc_read_t                     = ADCRead(self.serial_obj)
        self.adc_read_t.done_s              = self.console_s

        # - DAC Control
        self.dac_control_t                  = DACControl(self.serial_obj)
        self.dac_control_t.done_s           = self.console_s

        # - Reference Resistor Control
        self.reference_resistor_t           = ReferenceResistor(self.serial_obj)
        self.reference_resistor_t.done_s    = self.console_s

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
    # UI status ----------------------------------------------------------------------------------------------------------------
    # - True: Busy
    # - False: Idle
    def ui_set_serial_status(self, status):
        self.serial_status_title_label.setText('Busy' if status else 'Idle')
        ui_set_color_red(self.serial_status_title_label) if status else \
        ui_set_color_green(self.serial_status_title_label)

    def ui_get_serial_status(self):
        return False if self.serial_status_title_label.text()=='Idle' else True

    # UI file dialog -----------------------------------------------------------------------------------------------------------
    def ui_select_file(self, path_init='C:/'):
        return QFileDialog.getOpenFileName(self, "Select File", path_init)

    def ui_select_folder(self, path_init='C:/'):
        return QFileDialog.getExistingDirectory(self, "Select Folder", path_init)

    # ==========================================================================================================================
    # UI - File and Folder
    # ==========================================================================================================================
    # Create experiment folder depends on the date and user information --------------------------------------------------------
    def create_experiment_folder(self, folder_path, ask=True):
        folder_name = self.get_folder_name()
        create_folder_if_not_existed(folder_path, folder_name, ask=ask)
        folder_path = folder_path + '/' + folder_name
        return folder_path

    # Save logs ----------------------------------------------------------------------------------------------------------------
    def save_log(self, log_path):
        try:
            time_info = get_date_time(2)                # Date and time
            log_write = self.console_textEdit.toPlainText()   # Log
            self.console_textEdit.clear()               # Clear

            file = open(log_path + '/' + '%s_instrument_log.txt' % time_info, 'a')
            file.write('-'*32 + '\n')
            file.write(time_info + '\n')
            file.write('-'*32 + '\n')
            file.write(log_write)
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
        self.ui_status_update(None)

    def run_if_ready_else_exit(self, check_avail=1):
        # Check if serial is busy
        if self.ui_get_serial_status():
            ui_send_error('Serial is busy, you must stop first !')
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

            ui_send_info('%d valid devices found !\n' % len(port_list))
        except:
            ui_send_error("Error: fail to list serial port")

    def serial_open_port_done(self, data):
        try:
            if not data:
                self.serial_port_comboBox.setCurrentIndex(len(self.serial_name)-1)
            else:
                [ver, device_id, div] = data
                # UI
                self.clk_status_title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                self.clk_status_title_label.setText('%.2f MHz' % (72.0 / (1 << div)))
                ui_send_info('Device detected ...\nFirmware:  %d\nClock: %.2f MHz' % (ver, 72.0 / (1<<div)))
        except:
            ui_send_error("Error: fail to open serial port")
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

    def get_folder_name(self):
        exp_id      = self.exp_id_spinBox.value()
        ttn_freq    = int(72000 / (8 << self.clk_status_title_label.currentIndex()))
        user_com    = self.user_com_lineEdit.text()
        folder_name = 'D%s_E%s_F%sKHz' % (get_date(), str(exp_id).zfill(2), str(ttn_freq).zfill(4))
        if user_com!= '':
            for n in range(len(user_com)):
                user_chr = user_com[n]
                if not (user_chr.isdigit() or user_chr.isalpha() or user_chr == '_'):
                    ui_send_error("Only alphabet, digital and underscore are allowed for user comment !")
                    raise SystemError
            folder_name = folder_name + '_U_' + user_com
        return folder_name

    def ui_status_update(self, status):
        # Reset
        if status is None:
            self.pwr_status_title_12V_label.setText('N/A')
            self.pwr_status_title_neg12V_label.setText('N/A')
            self.pwr_status_title_3V3_label.setText('N/A')
            ui_set_color_red(self.pwr_status_title_12V_label)
            ui_set_color_red(self.pwr_status_title_neg12V_label)
            ui_set_color_red(self.pwr_status_title_3V3_label)
            return

        # Update
        status_elec_12v     = status[0]
        status_elec_neg12v  = status[1]
        status_elec_3v3     = status[2]

        if status_elec_12v:
            self.pwr_status_title_12V_label.setText('Active')
            ui_set_color_green(self.pwr_status_title_12V_label)
        else:
            self.pwr_status_title_12V_label.setText('Inactive')
            ui_set_color_red(self.pwr_status_title_12V_label)

        if status_elec_neg12v:
            self.pwr_status_title_neg12V_label.setText('Active')
            ui_set_color_green(self.pwr_status_title_neg12V_label)
        else:
            self.pwr_status_title_neg12V_label.setText('Inactive')
            ui_set_color_red(self.pwr_status_title_neg12V_label)

        if status_elec_3v3:
            self.pwr_status_title_3V3_label.setText('Active')
            ui_set_color_green(self.pwr_status_title_3V3_label)
        else:
            self.pwr_status_title_3V3_label.setText('Inactive')
            ui_set_color_red(self.pwr_status_title_3V3_label)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setFont(QFont("Consolas", 10))
    # Enable Fusion style (better dark theme support)
    app.setStyle("Fusion")
    ui = InstrumentMain()
    ui.show()
    sys.exit(app.exec())

