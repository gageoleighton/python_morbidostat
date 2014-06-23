import matplotlib.pyplot as plt
import numpy as np
import sys,os, glob, threading, time
import wx

class morbidostat_monitor(object):
    def __init__(self,dir_name):
        if os.path.exists(dir_name):
            self.data_dir = dir_name.rstrip('/')+'/'
            self.read_parameters_file()
            self.data_fig_name = 'Morbidostat'
            self.scan_dt = 10
            self.OD_dir = self.data_dir+'OD/'
            self.lock_file = self.data_dir+'.lock'
            self.data_range = 60*60
            self.time_unit = (60,'m')
            plt.ioff()
            self.init_data_plot()
            self.figure_updater = threading.Thread(target = self.update_cycle)
            self.figure_updater.daemon=True
        else:
            print "data directory not found"
    
    def start(self):
        self.continue_plotting=True
        self.figure_updater.start()

    def stop(self):
        self.continue_plotting=False
        self.figure_updater.join()

    def update_cycle(self):
        while self.continue_plotting:
            self.update_all()
            time.sleep(self.scan_dt)

    def update_all(self):
        self.load_cycle_data()
        self.load_OD_data()
        wx.CallAfter(self.update_plot)
       
    def read_parameters_file(self):
        try:
            with open(self.data_dir+'parameters.dat', 'r') as params_file:
                for line in params_file:
                    entries = line.split()
                    if entries[0]=='vials':
                        self.__setattr__('vials', map(int, entries[1:]))
                        self.n_vials = len(self.vials)
        except:
            print "can't read parameters file"
    
    def load_OD_data(self):
        if not os.path.exists(self.lock_file):
            OD_file_list = glob.glob(self.OD_dir+'OD_cycle*dat')
            if len(OD_file_list):
                tmp = np.loadtxt(OD_file_list[0])
                self.ODs_per_cycle, ncols = tmp.shape
                self.OD = np.zeros((len(OD_file_list)*self.ODs_per_cycle, ncols))
                for fname in OD_file_list:
                    cycle= int(fname.split('_')[-1][:-4])
                    if cycle<len(OD_file_list):
                        self.OD[self.ODs_per_cycle*cycle:self.ODs_per_cycle*(cycle+1),:]=np.loadtxt(fname)
                    else:
                        print "Cycle out of range"
            else:
                print "no OD data"
        else:
            print "data locked"


    def load_cycle_data(self):
        if not os.path.exists(self.lock_file):
            self.drug_concentration = np.loadtxt(self.data_dir+'vials_drug_concentrations.txt')
            self.temperature= np.loadtxt(self.data_dir+'temperature.txt')
            self.growth_rate_estimate = np.loadtxt(self.data_dir+'growth_rate_estimates.txt')
            self.OD_estimate = np.loadtxt(self.data_dir+'cycle_OD_estimate.txt')
        else:
            print "data locked"


    def init_data_plot(self):
        '''
        this function sets up the plot of the OD and the antibiotic concentration
        in each of the 
        '''
        print "init figure"
        # there is a subplot for each vial which share axis. hence they are stacked
        n_cols = 3 # subplots are arranged in rows of 3
        n_rows = int(np.ceil(1.0*self.n_vials/n_cols))+1
        figsize = (n_cols*2.5, n_rows*2)
        plt.figure(self.data_fig_name, figsize)
        plt.subplots_adjust(hspace = .001, wspace = .001)
        plt.suptitle('OD long term')
        vi = 0
        self.subplots = {}
        self.plotted_line_objects = {}
        for vi, vial  in enumerate(self.vials):
            self.subplots[vi] = [plt.subplot(n_rows, n_cols, vi+1)]
            self.subplots[vi].append(self.subplots[vi][0].twinx())
            self.plotted_line_objects[vi] = [self.subplots[vi][0].plot([],[], c='b')[0], 
                                             self.subplots[vi][1].plot([],[], c='r', marker='o')[0]]
            self.subplots[vi][0].text(0.1, 0.9, 'vial '+str(vial+1), transform=self.subplots[vi][0].transAxes)
            #       self.subplots[vi][0].plot([0,self.experiment_duration], [self.target_OD, self.target_OD], ls='--', c='k')

            # set up the axis such that the only the left most axis shows OD and ticks
            # and only the rightmost axis shows the antibiotic and ticks
            if vi%n_cols:
                self.subplots[vi][0].set_yticklabels([])
            else:
                self.subplots[vi][0].set_ylabel('OD')
            if vi%n_cols<n_cols-1:
                self.subplots[vi][1].set_yticklabels([])
            else:
                self.subplots[vi][1].set_ylabel('antibiotic')

            # switch off x tick labels in all but the bottom axis
            #if int(np.ceil(1.0*self.n_vials/n_cols))<n_rows-1:
            if vi < len(self.vials)-n_cols-1:
                self.subplots[vi][0].set_xticklabels([])
            else:
                self.subplots[vi][0].set_xlabel('time ['+self.time_unit[1]+']')
        self.subplots['temperature'] = plt.subplot2grid((n_rows, n_cols),(n_rows-1, 0), colspan=1)
        plt.xlabel('Time ['+self.time_unit[1]+']')
        plt.ylabel('Temperature [C]')
        self.temperature_curves = [plt.plot([],[],c='b', label = 'T1')[0], plt.plot([],[],c='r', label = 'T2')[0]]
        plt.legend()

        plt.show()

    def update_plot(self):
        '''
        this function is called repeatedly and redraws the plot after adding new data
        the axis are rescaled but kept identical in each subplot
        '''
        fig = plt.figure(self.data_fig_name)
        n_cycles_to_show = 5
        display_unit = 60
        max_OD = 0
        max_AB = 0
        x_data = self.OD[:,-1]
        x_alt_data = self.drug_concentration[:,-1]
        for vi, vial  in enumerate(self.vials):
            y_data = self.OD[:,vi]
            y_alt_data = self.drug_concentration[:,vi]
            max_OD = max(max_OD, y_data.max())
            max_AB = max(max_AB, y_alt_data.max())
            self.plotted_line_objects[vi][0].set_data(x_data/self.time_unit[0], y_data)
            self.plotted_line_objects[vi][1].set_data(x_alt_data/self.time_unit[0], y_alt_data)

        xmax = x_data.max()
        if self.data_range==0:
            xmin=0
        else:
            xmin = max(0,xmax-self.data_range)
        for vi, vial  in enumerate(self.vials):
            self.subplots[vi][0].set_ylim([0,max_OD*1.2+0.01])
            self.subplots[vi][0].set_xlim([xmin/self.time_unit[0], xmax/self.time_unit[0]])
            self.subplots[vi][1].set_ylim([0,max_AB*1.2+0.01])

        # temperature plot
        zero_indices = np.where(self.temperature[:,-1]==0)[0]
        if len(zero_indices)>1:
            max_index=zero_indices[1]-1
        else:
            max_index = self.temperature.shape[0]
        self.temperature_curves[0].set_data(self.temperature[:max_index,-1]/self.time_unit[0], self.temperature[:max_index,0])
        self.temperature_curves[1].set_data(self.temperature[:max_index,-1]/self.time_unit[0], self.temperature[:max_index,1])
        plt.xlim(xmin/self.time_unit[0], self.temperature[:max_index,-1].max()/self.time_unit[0])
        plt.ylim(self.temperature[:max_index,:2].flatten().min()-2, self.temperature[:max_index,:2].flatten().max()+2)
        plt.draw()


if __name__ == '__main__':
    if len(sys.argv):
        morb_monitor = morbidostat_monitor(sys.argv[1])
        time.sleep(3)
        morb_monitor.start()
    else:
        print "name of data directory required"
