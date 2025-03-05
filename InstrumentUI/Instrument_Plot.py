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
        self.setGeometry(100, 100, 4*88, 8*88)
        #self.setFixedSize(350, 531)
        
        # Matplotlib inst
        self.canvas = PlotCanvas(self)
        self.canvas.move(0, 0)
        
        # Window state
        self.window_existed = False
        
        # Test
        # - Imaging
        img = self.test_img_gen()
        self.canvas.plot_image(img)
        # - Readout curve
        cur = list(range(0, 10))
        self.canvas.plot_cur(cur)
        # - Temperature curve
        tem = list(range(0, 10))
        self.canvas.plot_tem(tem)

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
    
    tem_ref  = 0
    img_vmax = 1023
    img_cbar = None

    def __init__(self, parent=None):
        # Tune dpi so that temperature pixels are well meshed
        self.figure = Figure(figsize=(4, 9), dpi=88)
        FigureCanvas.__init__(self, self.figure)
        self.setParent(parent)
        
        # Canvas Grid
        self.grid = gridspec.GridSpec(20, 10, hspace=2)

        # Imaging
        self.img_data = np.zeros(290*204).reshape(290, 204)
        self.img_ax   = self.figure.add_subplot(self.grid[0:9, :])
        self.img_fig  = self.img_ax.imshow( self.img_data,
                                                    #cmap=plt.get_cmap('hsv'),
                                                    cmap=plt.get_cmap('gist_rainbow'), 
                                                    interpolation='none',
                                                    #extent=[0,  77,  0, 55], 
                                                    vmin=0,  vmax=2047)

        self.img_ax.set_title("IMAGING", pad=10)
        self.img_ax.set_xlabel("COLUMN")
        self.img_ax.set_ylabel("ROW")
        
        # - Color bar
        cbar_div = make_axes_locatable(self.img_ax)
        cbar_cax = cbar_div.append_axes("right", size="5%", pad=0.1)
        self.img_cbar = self.figure.colorbar(self.img_fig,  orientation='vertical', cax=cbar_cax)
        
        # Readout Curve
        self.cur_data = 0
        self.cur_ax   = self.figure.add_subplot(self.grid[10:16, 1:])
        self.cur_ax.yaxis.set_label_position("right")
        self.cur_ax.set_ylabel('Average', labelpad=10)
        self.cur_ax.set_xlim(0, 1)
        self.cur_ax.set_ylim(0, 1023)
        [self.cur_fig] = self.cur_ax.plot([self.cur_data], 'b-')
        
        # Temperature Curve
        self.tem_data = 0
        self.tem_ax   = self.figure.add_subplot(self.grid[17:20, 1:])
        self.tem_ax.yaxis.set_label_position("right")
        self.tem_ax.set_ylabel('Temp', labelpad=10)
        self.tem_ax.set_xlabel('Time [s]', labelpad=10)
        self.tem_ax.set_xlim(0, 1)
        self.tem_ax.set_ylim(10, 75)
        [self.tem_fig]     = self.tem_ax.plot([self.tem_data], 'b-')
        [self.tem_ref_fig] = self.tem_ax.plot([self.tem_ref], 'r-')

        # Draw
        self.draw()
        
    def drawnow(self):
        self.draw()

    def plot_image(self, data, vmax=1023):
        # Remove colorbar
        if(self.img_vmax!=vmax):
            try:
                self.img_cbar.remove()
            except:
                pass
 
        self.img_fig.remove()
        self.img_data = np.array(data).reshape(204, 290)
        self.img_data = self.img_data.transpose()
        self.img_fig  = self.img_ax.imshow( self.img_data,
                                                    #cmap=plt.get_cmap('hsv'),
                                                    cmap=plt.get_cmap('gist_rainbow'), 
                                                    interpolation='none',
                                                    #extent=[0,  77,  0, 55], 
                                                    vmin=0,  vmax=vmax)
        # Update colorbar
        if(self.img_vmax!=vmax):
            self.img_vmax = vmax
            
            cbar_div = make_axes_locatable(self.img_ax)
            cbar_cax = cbar_div.append_axes("right", size="5%", pad=0.1)
            self.img_cbar = self.figure.colorbar(self.img_fig,  orientation='vertical', cax=cbar_cax)
            self.draw()
        
    def plot_cur(self, data, timestamp=None):
        self.cur_fig.remove()
        self.cur_ax.clear() #!!!
        self.cur_data = np.array(data)
        self.cur_ax.set_ylim(np.min(self.cur_data)-10, np.max(self.cur_data)+10)
        
        if(timestamp!=None):
            self.cur_ax.set_xlim(0, timestamp[-1])
            if(len(self.cur_data.shape)==1):
                [self.cur_fig] = self.cur_ax.plot(timestamp, self.cur_data, 'b-')
            else:
                for n in range(0, self.cur_data.shape[1]):
                    [self.cur_fig] = self.cur_ax.plot(timestamp, self.cur_data[:, n], label='W %d' % n)
                #if(self.cur_data.shape[0]==1):
                #    self.cur_ax.legend()
        else:
            x_len = len(self.cur_data)
            self.cur_ax.set_xlim(0, x_len)
            if(len(self.cur_data.shape)==1):
                [self.cur_fig] = self.cur_ax.plot(self.cur_data, 'b-')
            else:
                for n in range(0, self.cur_data.shape[1]):
                    [self.cur_fig] = self.cur_ax.plot(self.cur_data[:, n], label='W %d' % n)
                #if(self.cur_data.shape[0]==1):
                #    self.cur_ax.legend()
    
        # Text
        #self.cur_ax.text(6, 10, '%4d' % data[-1], fontsize = 22)
        #self.draw()
        
    def set_tem_ref(self, val):
        self.tem_ref = val
        
    def plot_tem(self, data, timestamp=None):
        self.tem_fig.remove()
        self.tem_ref_fig.remove()
        
        self.tem_data = data
        x_len = len(self.tem_data)
        self.tem_ax.set_xlim(0, x_len)
        self.tem_ax.set_ylim(min(self.tem_data)-10, max(max(self.tem_data), self.tem_ref)+10)
        
        if(timestamp!=None):
            self.tem_ax.set_xlim(0, timestamp[-1])
            [self.tem_fig]     = self.tem_ax.plot(timestamp, self.tem_data, 'b-')
            [self.tem_ref_fig] = self.tem_ax.plot(timestamp, [self.tem_ref]*x_len, 'r-')
        else:
            self.tem_ax.set_xlim(0, x_len)
            [self.tem_fig] = self.tem_ax.plot(self.tem_data, 'b-')
            [self.tem_ref_fig] = self.tem_ax.plot([self.tem_ref]*x_len, 'r-')

        # Text
        #self.tem_ax.text(6, 10, '%4d' % data[-1], fontsize = 22)
        #self.draw()

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
