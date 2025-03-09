# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from PyQt6.QtWidgets import QMainWindow, QApplication
#from PyQt6.QtWidgets import QSizePolicy

# --------------------------------------------------------------
# Plot Import
# --------------------------------------------------------------
import matplotlib
matplotlib.use("Qt5Agg")

import matplotlib.pyplot as plt

from matplotlib import gridspec
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.axes_grid1 import make_axes_locatable

# --------------------------------------------------------------
# System Import
# --------------------------------------------------------------
import numpy as np

# --------------------------------------------------------------
# Plot Class
# --------------------------------------------------------------
# Plot window for test
class Plot(QMainWindow):

    def __init__(self):
        super().__init__()
        # Window init
        self.setWindowTitle('Plot')
        self.setGeometry(100, 100, 440, 890)
        #self.setFixedSize(350, 531)
        
        # Matplotlib inst
        self.canvas = PlotCanvas(self)
        self.canvas.move(0, 0)
        
        # Window state
        self.window_existed = False

        # Test
        # Generate sample data
        # - Bode plot (Magnitude and Phase)
        freq = np.logspace(1, 5, 100)  # Frequency range from 10 Hz to 100 kHz
        mag = 20 * np.log10(1 / (1 + (freq / 1000) ** 2))  # Example low-pass filter response
        phase = -np.arctan(freq / 1000) * (180 / np.pi)  # Corresponding phase shift

        self.canvas.plot_bode(freq, mag, phase)

        # - RLC Fit Curve and Error
        time = np.linspace(0, 5, 100)  # Time range from 0 to 5s
        rlc_fit = np.exp(-time) * np.cos(2 * np.pi * time)  # Example damped oscillation
        error = np.random.normal(0, 0.05, size=time.shape)  # Small random noise as error

        self.canvas.plot_rlc_fit(time, rlc_fit, error)

        # - Q Factor Plot
        q_factor = 10 + 50 * np.exp(-freq / 10000)  # Example Q-factor variation with frequency
        self.canvas.plot_q_factor(freq, q_factor)

    def closeEvent(self, event):
        self.window_existed = False
        
    def show_window(self):
        self.show()
        self.window_existed = True
        
    def is_window_existed(self):
        return self.window_existed
 
    def save_image(self, path):
        self.canvas.save_image(path)
        
    def test_img_gen(self):
        img = []
        # Temperature pixel if (r%5==2 and c%5==2)
        for c in range(0, 204):
            for r in range(0, 290):
                img.append(511 if (r%5==2 and c%5==2) else 0)
        return img
                
class PlotCanvas(FigureCanvas):

    def __init__(self, parent=None):
        # Initialize the figure with a 5x10-inch size and 88 DPI
        self.figure = Figure(figsize=(5, 10), dpi=88)
        FigureCanvas.__init__(self, self.figure)
        self.setParent(parent)

        # Define a grid layout with 20 rows and 10 columns
        self.grid = gridspec.GridSpec(46, 20)

        # -------------------------------------------------
        # 1. Bode Plot (Magnitude & Phase) - Top section
        # -------------------------------------------------
        # Create an axis for the Bode magnitude plot
        self.bode_mag_ax = self.figure.add_subplot(self.grid[0:9, 1:20])
        self.bode_mag_ax.set_title("Magnitude", fontdict={'fontweight': 'bold'})
        # self.bode_mag_ax.set_xlabel("Frequency [Hz]", labelpad=-1)
        self.bode_mag_ax.set_ylabel("Magnitude [dB]", labelpad=-2)
        self.bode_mag_ax.set_xscale("log")  # Logarithmic scale for frequency
        self.bode_mag_ax.set_ylim(-100, 20)  # Typical dB range
        [self.bode_mag_fig] = self.bode_mag_ax.plot([], [], 'b-')  # Initialize with an empty plot

        # Create an axis for the Bode phase plot below the magnitude plot
        self.bode_phase_ax = self.figure.add_subplot(self.grid[13:22, 1:20])
        self.bode_phase_ax.set_title("Phase", fontdict={'fontweight': 'bold'})
        # self.bode_phase_ax.set_xlabel("Frequency [Hz]", labelpad=-1)
        self.bode_phase_ax.set_ylabel("Phase [degrees]", labelpad=-2)
        self.bode_phase_ax.set_xscale("log")
        self.bode_phase_ax.set_ylim(-180, 180)  # Phase range
        [self.bode_phase_fig] = self.bode_phase_ax.plot([], [], 'r-')

        # -------------------------------------------------
        # 2. RLC Fit Curve & Error Plot - Middle section
        # -------------------------------------------------
        # Create an axis for the RLC fit curve
        self.rlc_fit_ax = self.figure.add_subplot(self.grid[26:35, 1:8])
        self.rlc_fit_ax.set_title("RLC Fit", fontdict={'fontweight': 'bold'})
        # self.rlc_fit_ax.set_xlabel("Frequency [Hz]", labelpad=-1)
        self.rlc_fit_ax.set_ylabel("Voltage [V]", labelpad=-2)
        self.rlc_fit_ax.set_xscale("log")
        self.rlc_fit_ax.set_ylim(-1, 1)  # Placeholder range
        [self.rlc_fit_fig] = self.rlc_fit_ax.plot([], [], 'g-')  # Empty initial plot

        # Create an axis for the error plot (positioned next to the RLC fit plot)
        self.rlc_error_ax = self.figure.add_subplot(self.grid[26:35, 12:20])
        self.rlc_error_ax.set_title("Fit Error", fontdict={'fontweight': 'bold'})
        # self.rlc_error_ax.set_xlabel("Frequency [Hz]", labelpad=-1)
        self.rlc_error_ax.set_ylabel("Error [%]", labelpad=-2)
        self.rlc_error_ax.set_xscale("log")
        self.rlc_error_ax.set_ylim(-0.1, 0.1)  # Placeholder range
        [self.rlc_error_fig] = self.rlc_error_ax.plot([], [], 'm-')  # Empty initial plot

        # -------------------------------------------------
        # 3. Q Factor Plot - Bottom section
        # -------------------------------------------------
        self.q_factor_ax = self.figure.add_subplot(self.grid[39:46, :])
        self.q_factor_ax.set_title("Q Factor", fontdict={'fontweight': 'bold'})
        # self.q_factor_ax.set_xlabel("Frequency [Hz]", labelpad=-1)
        # self.q_factor_ax.set_ylabel("Q Factor")
        self.q_factor_ax.set_xscale("log")  # Logarithmic scale for frequency
        self.q_factor_ax.set_ylim(0, 100)  # Placeholder range
        [self.q_factor_fig] = self.q_factor_ax.plot([], [], 'c-')  # Empty initial plot

        # Render the initial empty plots
        self.figure.subplots_adjust(right=0.95, top=0.95, bottom=0.05, hspace=0.3)
        self.draw()
        
    def drawnow(self):
        self.draw()

    def plot_bode(self, frequency, magnitude, phase):
        """
        Updates the Bode plot with new magnitude and phase data.

        Parameters:
        - frequency: List or numpy array of frequency values (log scale).
        - magnitude: Corresponding magnitude values in dB.
        - phase: Corresponding phase values in degrees.
        """

        # Update magnitude plot
        self.bode_mag_fig.remove()
        self.bode_mag_fig, = self.bode_mag_ax.plot(frequency, magnitude, 'b-')
        self.bode_mag_ax.relim()  # Adjust limits based on data
        self.bode_mag_ax.autoscale_view()

        # Update phase plot
        self.bode_phase_fig.remove()
        self.bode_phase_fig, = self.bode_phase_ax.plot(frequency, phase, 'r-')
        self.bode_phase_ax.relim()
        self.bode_phase_ax.autoscale_view()

        # Redraw the figure
        self.figure.subplots_adjust(right=0.95, top=0.95, bottom=0.05, hspace=0.3)
        self.draw()

    def plot_rlc_fit(self, time, fit_curve, error):
        """
        Updates the RLC fit curve and the corresponding error plot.

        Parameters:
        - time: List or numpy array of time values.
        - fit_curve: Corresponding voltage values for the RLC fit.
        - error: Error between measured and fitted values.
        """

        # Update RLC Fit Curve
        self.rlc_fit_fig.remove()
        self.rlc_fit_fig, = self.rlc_fit_ax.plot(time, fit_curve, 'g-')
        self.rlc_fit_ax.relim()
        self.rlc_fit_ax.autoscale_view()

        # Update Fit Error Plot
        self.rlc_error_fig.remove()
        self.rlc_error_fig, = self.rlc_error_ax.plot(time, error, 'm-')
        self.rlc_error_ax.relim()
        self.rlc_error_ax.autoscale_view()

        # Redraw the figure
        self.figure.subplots_adjust(right=0.95, top=0.95, bottom=0.05, hspace=0.3)
        self.draw()

    def plot_q_factor(self, frequency, q_factor):
        """
        Updates the Q factor plot.

        Parameters:
        - frequency: List or numpy array of frequency values (log scale).
        - q_factor: Corresponding Q factor values.
        """

        # Update Q Factor Plot
        self.q_factor_fig.remove()
        self.q_factor_fig, = self.q_factor_ax.plot(frequency, q_factor, 'c-')
        self.q_factor_ax.relim()
        self.q_factor_ax.autoscale_view()

        # Redraw the figure
        self.figure.subplots_adjust(right=0.95, top=0.95, bottom=0.05, hspace=0.3)
        self.draw()

    # Save plot to .png
    def save_png(self, path):
        self.figure.savefig(path)

# --------------------------------------------------------------
# Entry
# --------------------------------------------------------------
if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    ui = Plot()
    ui.show_window()
    sys.exit(app.exec())
