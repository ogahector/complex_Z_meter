# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from Ui_Instrument_Main import Ui_InstrumentMain

from PyQt6.QtWidgets import QMainWindow, QApplication
from PyQt6.QtWidgets import QFileDialog, QCompleter
from PyQt6.QtWidgets import QTableWidgetItem
from PyQt6.QtCore import pyqtSlot, pyqtSignal, Qt
from PyQt6.QtGui import QTextCursor, QColor, QFont

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from Instrument_Plot import PlotCanvas
from Instrument_Cmd       import *
# from TEST_Instrument_Cmd import *
from Instrument_Thread import *
from Instrument_Func import *


class InstrumentMain(QMainWindow, Ui_InstrumentMain):
    # Signal
    serial_status = pyqtSignal(bool)
    console_s = pyqtSignal(str)

    msgbox_error_s = pyqtSignal(str)
    msgbox_info_s = pyqtSignal(str)

    # Variable
    serial_obj = None  # Instance of Serial
    serial_name = [None, 'Refresh']  # List of serial com port
    serial_port = None

    def __init__(self, parent=None):
        super(InstrumentMain, self).__init__(parent)
        self.setupUi(self)

        # Get UI System Parameter
        self.ui_cwd = os.getcwd()

        # Initial UI size --------------------------------------------------------------------------
        self.setFixedSize(790, 910)

        # - Auto complete user command
        self.user_cmd_lineEdit.setCompleter(QCompleter(command_list, self.user_cmd_lineEdit))

        # Plot Canvas ----------------------------------------------------------------------------------------------------------
        self.plot_obj = PlotCanvas(self)
        self.plot_obj.move(335, 20)

        # Serial ---------------------------------------------------------------------------------------------------------------
        # self.serial_obj = TESTInstrumentCmd()
        self.serial_obj           = DebugCommand()
        self.serial_obj.status_s = self.serial_status
        self.serial_obj.console_s = self.console_s
        self.serial_obj.msgbox_s = self.msgbox_error_s

        # Shared System signals ------------------------------------------------------------------------------------------------
        self.serial_status.connect(self.ui_set_serial_status)
        self.console_s.connect(self.ui_print)
        self.msgbox_error_s.connect(ui_send_error)
        self.msgbox_info_s.connect(ui_send_info)

        # Threads --------------------------------------------------------------------------------------------------------------
        # - List available serial ports
        self.serial_list_port_t = SerialListPort(self.serial_obj)
        self.serial_list_port_t.done_s.connect(self.serial_list_port_done)
        self.serial_list_port_t.start()

        # - Open serial port
        self.serial_open_port_t = SerialOpenPort(self.serial_obj)
        self.serial_open_port_t.done_s.connect(self.serial_open_port_done)

        # - Serial general command
        self.serial_general_cmd_t = SerialGeneralCmd(self.serial_obj)
        self.serial_general_cmd_t.done_s = self.console_s

        # - Real-time Current Sensing
        self.current_read_t = CurrentRead(self.serial_obj)
        self.current_read_t.done_s = self.console_s

        # - Temperature Readout
        self.adc_read_t = ADCRead(self.serial_obj)
        self.adc_read_t.done_s = self.console_s

        # - DAC Control
        self.dac_control_t = DACControl(self.serial_obj)
        self.dac_control_t.done_s = self.console_s

        # - Reference Resistor Control
        self.reference_resistor_t = ReferenceResistor(self.serial_obj)
        self.reference_resistor_t.done_s = self.console_s

        # Main process ---------------------------------------------------------------------------------------------------------
        # - Init
        self.init_t = Init(self.serial_obj)
        self.init_t.done_s.connect(self.init_done)

        # - Short Calibration
        self.sc_t = ShortCalibration(self.serial_obj, self.plot_obj, self.plot_obj)
        self.sc_t.done_s.connect(self.sc_step_done)

        # - Open Calibration
        self.oc_t = OpenCalibration(self.serial_obj, self.plot_obj, self.plot_obj)
        self.oc_t.done_s.connect(self.oc_step_done)

        # - Load Calibration
        self.ld_t = LoadCalibration(self.serial_obj, self.plot_obj, self.plot_obj)
        self.ld_t.done_s.connect(self.ld_step_done)

        # - Measurement Readout
        self.meas_t = ReadoutMeasurement(self.serial_obj, self.plot_obj)
        self.meas_t.done_s.connect(self.meas_step_done)

        # - RLC Fitting
        self.rlc_t = RLCFitting(self.serial_obj, self.plot_obj, self.plot_obj)
        self.rlc_t.done_s.connect(self.meas_step_done)

        # - Q Factor
        self.qf_t = QFactor(self.serial_obj, self.plot_obj, self.plot_obj)
        self.qf_t.done_s.connect(self.meas_step_done)

        # User interface -------------------------------------------------------------------------------------------------------
        # - TTN Check Status
        self.ttn_check_status_t = CheckPowerStatus(self.serial_obj)
        self.ttn_check_status_t.done_s.connect(self.check_status_done)

    def closeEvent(self, event):
        if not ui_send_question('Are you sure to quit ?'):
            event.ignore()
        else:
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
        return False if self.serial_status_title_label.text() == 'Idle' else True

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
            time_info = get_date_time(2)  # Date and time
            log_write = self.console_textEdit.toPlainText()  # Log
            self.console_textEdit.clear()  # Clear

            file = open(log_path + '/' + '%s_instrument_log.txt' % time_info, 'a')
            file.write('-' * 32 + '\n')
            file.write(time_info + '\n')
            file.write('-' * 32 + '\n')
            file.write(log_write)
            file.close()

        except Exception:
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
        self.serial_general_cmd_t.command = command
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
        except Exception:
            pass

    # ================================================================================================================================
    # UI - Update and Control
    # ================================================================================================================================
    def reset_ui(self):
        self.ui_status_update(None)

    def run_if_ready_else_exit(self):
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
            self.serial_port_comboBox.setCurrentIndex(len(self.serial_name) - 1)

            ui_send_info('%d valid devices found !\n' % len(port_list))
        except Exception:
            ui_send_error("Error: fail to list serial port")

    def serial_open_port_done(self, data):
        try:
            if not data:
                self.serial_port_comboBox.setCurrentIndex(len(self.serial_name) - 1)
            else:
                [ver, device_id, clk] = data
                # UI
                self.clk_status_title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                self.clk_status_title_label.setText('%.2f MHz' % clk)
                ui_send_info('Device detected ...\nFirmware:  %d\nClock: %.2f MHz' % (ver, clk))
        except Exception:
            ui_send_error("Error: fail to open serial port")
            self.serial_port_comboBox.setCurrentIndex(len(self.serial_name) - 1)

    @pyqtSlot(int)
    def on_serial_port_comboBox_activated(self, index):
        if self.serial_obj.is_connected():
            self.serial_obj.close_serial()  # Close serial
            self.reset_ui()  # Reset ui components

        if len(self.serial_name) == index + 1:
            self.serial_list_port_t.start()
        else:
            self.serial_open_port_t.config(self.serial_port[index])
            self.serial_open_port_t.start()

    @pyqtSlot()
    def on_path_toolButton_clicked(self):
        init_path = self.path_lineEdit.text()
        folder_path = self.ui_select_folder(init_path)
        if self.ui_cwd.replace('/', '\\') in folder_path:
            folder_path = folder_path.replace(self.ui_cwd.replace('/', '\\'), '.')
        elif self.ui_cwd.replace('\\', '/') in folder_path:
            folder_path = folder_path.replace(self.ui_cwd.replace('\\', '/'), '.')
        self.path_lineEdit.setText(folder_path if folder_path else init_path)

    def get_folder_name(self):
        exp_id = self.exp_id_spinBox.value()
        user_com = self.user_com_lineEdit.text()
        folder_name = 'D%s_E%s' % (get_date(), str(exp_id).zfill(2))
        if user_com != '':
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
        status_elec_12v = status[0]
        status_elec_neg12v = status[1]
        status_elec_3v3 = status[2]

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

    def check_status_done(self, data_list):
        # Update button text and show result
        self.pwr_status_check_pushButton.setText('Check')
        if data_list:
            status_elec_12v = data_list[0]
            status_elec_neg12v = data_list[1]
            status_elec_3v3 = data_list[2]
            self.ui_status_update([status_elec_12v == 1, status_elec_neg12v == 1, status_elec_3v3 == 1])
            # Elec
            if status_elec_12v == 0 or status_elec_neg12v == 0 or status_elec_3v3 == 0:
                ui_send_error('PCB doesn\'t have correct voltage supplies :(')
                return
        else:
            self.ui_status_update(None)

    @pyqtSlot()
    def on_pwr_status_check_pushButton_clicked(self):
        try:
            if self.pwr_status_check_pushButton.text() == 'Check':
                # Check if free to run
                self.run_if_ready_else_exit()
                # Update label
                self.pwr_status_check_pushButton.setText('Running')
                # Start
                self.ttn_check_status_t.start()
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    ############################################################################################################################
    # Main Process
    ############################################################################################################################
    # Init
    # --------------------------------------------------------------------------------------------------------------------------
    def enter_init_step(self, file_path=None):
        # UI
        self.init_pushButton.setText('Running')
        ui_set_color_red(self.init_pushButton)

        # Thread
        self.init_t.config(file_path)
        self.init_t.start()

    def leave_init_step(self):
        # UI
        self.init_pushButton.setText('Init')
        ui_set_color_green(self.init_pushButton)

    def init_done(self, data_list):
        # Check if thread error
        if data_list:
            file_path = data_list[0]
            status = data_list[1]
            # Update UI
            self.ui_status_update([status, status, status])
            # Next step
            if status:
                if ui_send_question('Start short circuit calibration?'):
                    self.leave_init_step()
                    self.enter_sc_step(file_path)
                else:
                    self.leave_init_step()
                    ui_send_info('PCB has correct voltage supplies :)')
                return
            # PCB is inactive
            else:
                ui_send_error('PCB doesn\'t have correct voltage supplies :(')

            self.leave_init_step()

        else:
            self.ui_status_update(None)
            self.leave_init_step()

    @pyqtSlot()
    def on_init_pushButton_clicked(self):
        try:
            if self.init_pushButton.text() == 'Init':
                # Check if free to run
                self.run_if_ready_else_exit()
                # Create experiment folder
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                # Start
                self.enter_init_step(file_path=file_path)
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # Short Calibration
    # --------------------------------------------------------------------------------------------------------------------------
    def enter_sc_step(self, file_path):
        # UI
        self.sc_pushButton.setText('Running')
        ui_set_color_red(self.sc_pushButton)
        # Thread
        self.sc_t.config(file_path)
        self.sc_t.start()

    def leave_sc_step(self):
        # UI
        self.sc_pushButton.setText('SC')
        ui_set_color_green(self.sc_pushButton)

    def sc_step_done(self, data_list):
        # Check if thread error
        if data_list:
            file_path = data_list[0]
            sc_phasors = data_list[1]  # Short circuit phasors

            # Next step
            if ui_send_question('Start open circuit calibration?'):
                self.leave_sc_step()
                self.enter_oc_step(file_path)
            else:
                self.leave_sc_step()
                ui_send_info('Short circuit calibration done :)')
        else:
            self.leave_sc_step()

    @pyqtSlot()
    def on_sc_pushButton_clicked(self):
        try:
            if self.sc_pushButton.text() != 'Running':
                # Check if free to run
                self.run_if_ready_else_exit()
                # Create experiment folder
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                # Start
                self.enter_sc_step(file_path)
            else:
                self.sc_t.stop()
                # ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # Open Calibration
    # --------------------------------------------------------------------------------------------------------------------------
    def enter_oc_step(self, file_path):
        # UI
        self.oc_pushButton.setText('Running')
        ui_set_color_red(self.oc_pushButton)
        # Thread
        self.oc_t.config(file_path)
        self.oc_t.start()

    def leave_oc_step(self):
        # UI
        self.oc_pushButton.setText('OC')
        ui_set_color_green(self.oc_pushButton)

    def oc_step_done(self, data_list):
        # Check if thread error
        if data_list:
            file_path = data_list[0]
            oc_phasors = data_list[1]  # Open circuit phasors

            # Next step
            if ui_send_question('Start measurement?'):
                self.leave_oc_step()
                self.enter_ld_step(file_path)
            else:
                self.leave_oc_step()
                ui_send_info('Open circuit calibration done :)')
        else:
            self.leave_oc_step()

    @pyqtSlot()
    def on_oc_pushButton_clicked(self):
        try:
            if self.oc_pushButton.text() == 'OC':
                # Check if free to run
                self.run_if_ready_else_exit()
                # Create experiment folder
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                # Start
                self.enter_oc_step(file_path)
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # Load Calibration
    # --------------------------------------------------------------------------------------------------------------------------
    def enter_ld_step(self, file_path):
        # UI
        self.ld_pushButton.setText('Running')
        ui_set_color_red(self.ld_pushButton)
        # Thread
        self.ld_t.config(file_path)
        self.ld_t.start()

    def leave_ld_step(self):
        # UI
        self.ld_pushButton.setText('Load')
        ui_set_color_green(self.ld_pushButton)

    def ld_step_done(self, data_list):
        # Check if thread error
        if data_list:
            file_path = data_list[0]
            oc_phasors = data_list[1]  # Open circuit phasors

            # Next step
            if ui_send_question('Start measurement?'):
                self.leave_ld_step()
                self.enter_meas_step(file_path)
            else:
                self.leave_ld_step()
                ui_send_info('Load circuit calibration done :)')
        else:
            self.leave_ld_step()

    @pyqtSlot()
    def on_ld_pushButton_clicked(self):
        try:
            if self.ld_pushButton.text() == 'Load':
                # Check if free to run
                self.run_if_ready_else_exit()
                # Create experiment folder
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                # Start
                self.enter_ld_step(file_path)
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # Measurement Step
    # --------------------------------------------------------------------------------------------------------------------------

    def enter_meas_step(self, file_path):
        # UI
        self.measure_pushButton.setText('Running')
        ui_set_color_red(self.measure_pushButton)
        # Thread
        self.meas_t.config(file_path)
        self.meas_t.start()

    def leave_meas_step(self):
        self.measure_pushButton.setText('Measure')
        ui_set_color_green(self.measure_pushButton)

    def meas_step_done(self, data_list):
        self.leave_meas_step()
        if data_list:
            file_path = data_list[0]
            meas_phasors = data_list[1]  # Measurement phasors

            if ui_send_question('Start RLC Fitting?'):
                self.leave_meas_step()
                self.enter_rlc_step(file_path)
            else:
                self.leave_oc_step()
                ui_send_info('Measurement stage done :)')
        else:
            self.leave_meas_step()

    @pyqtSlot()
    def on_measure_pushButton_clicked(self):
        try:
            if self.measure_pushButton.text() == 'Measure':
                self.run_if_ready_else_exit()
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                self.enter_meas_step(file_path)
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # RLC Fit Step
    # --------------------------------------------------------------------------------------------------------------------------

    def enter_rlc_step(self, file_path):
        # UI
        self.rlc_pushButton.setText('Running')
        ui_set_color_red(self.rlc_pushButton)
        # Thread
        self.rlc_t.config(file_path)
        self.rlc_t.start()

    def leave_rlc_step(self):
        self.rlc_pushButton.setText('RLC Fitting')
        ui_set_color_green(self.rlc_pushButton)

    def rlc_step_done(self, data_list):
        self.leave_rlc_step()
        if data_list:
            file_path = data_list[0]

            if ui_send_question('Start Q Factor calculation?'):
                self.leave_rlc_step()
                self.enter_qf_step(file_path)
            else:
                self.leave_oc_step()
                ui_send_info('Measurement stage done :)')
        else:
            self.leave_rlc_step()

    @pyqtSlot()
    def on_rlc_pushButton_clicked(self):
        try:
            if self.rlc_pushButton.text() == 'RLC Fit':
                self.run_if_ready_else_exit()
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                self.enter_rlc_step(file_path)
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # Q Factor Step
    # --------------------------------------------------------------------------------------------------------------------------

    def enter_qf_step(self, file_path):
        # UI
        self.qfactor_pushButton.setText('Running')
        ui_set_color_red(self.qfactor_pushButton)
        # Thread
        self.qf_t.config(file_path)
        self.qf_t.start()

    def leave_qf_step(self):
        self.qfactor_pushButton.setText('Q Factor')
        ui_set_color_green(self.qfactor_pushButton)

    def qf_step_done(self, data_list):
        self.leave_qf_step()
        if data_list:
            ui_send_info('Done :)')

    @pyqtSlot()
    def on_qfactor_pushButton_clicked(self):
        try:
            if self.qfactor_pushButton.text() == 'Q Factor':
                self.run_if_ready_else_exit()
                file_path = self.create_experiment_folder(self.path_lineEdit.text(), ask=False)
                self.enter_qf_step(file_path)
            else:
                ui_send_warning('Running, please wait...')
        except Exception:
            pass

    # --------------------------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    app.setFont(QFont("Consolas", 10))
    # Enable Fusion style (better dark theme support)
    app.setStyle("Fusion")
    ui = InstrumentMain()
    ui.show()
    sys.exit(app.exec())
