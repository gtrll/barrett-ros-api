import numpy as np
from os.path import join, exists
from os import makedirs
import matplotlib.pyplot as plt

def load_data(file_path, dof):
	dataMat=np.loadtxt(file_path,delimiter=',')
	data={}
	data['abs_time']=dataMat[:,0]
	data['ramp']=dataMat[:,1]
	data['jp_cmd']=180./np.pi*dataMat[:,2:2+dof]
	data['jp_meas']=180./np.pi*dataMat[:,2+dof:2+2*dof]
	data['jv_meas']=180./np.pi*dataMat[:,2+2*dof:2+3*dof]
	data['jt_meas']=dataMat[:,2+3*dof:2+4*dof]
	return data

def plot_data(data,out_dir,dof):

	for j_ind in range(dof):

		fig=plt.figure(figsize=(20,10))	
		ax11 = fig.add_subplot(3, 2, 1)
		ax12 = fig.add_subplot(3, 2, 2)
		ax21 = fig.add_subplot(3, 2, 3)
		ax22 = fig.add_subplot(3, 2, 4)
		ax31 = fig.add_subplot(3, 2, 5)

		ax11.plot(data['abs_time'][:],data['ramp'][:],'-k', markersize=2, label='ramp (sec.)')
		ax11.get_xaxis().set_visible(False)
		ax11.set_ylabel('ramp (sec)', fontsize=15)

		### First joint ####
		ax21.plot(data['abs_time'][:], data['jp_meas'][:,j_ind],'.k', markersize=2, label='jp_meas')
		ax21.plot(data['abs_time'][:], data['jp_cmd'][:,j_ind],'.r', markersize=2, label='jp_cmd')
		ax21.get_xaxis().set_visible(False)
		ax21.set_ylabel('jp_cmd (deg.)', fontsize=15)

		ax31.plot(data['abs_time'][:],data['jp_meas'][:,j_ind],'.k', markersize=2, label='jp_meas')
		ax31.set_xlabel('sys. time (sec)', fontsize=15)
		ax31.set_ylabel('jp_meas (deg.)', fontsize=15)

		ax12.plot(data['abs_time'][:],data['jv_meas'][:,j_ind],'.k', markersize=2, label='jv_meas')
		ax12.get_xaxis().set_visible(False)
		ax12.set_ylabel('jv_meas (deg./sec)', fontsize=15)

		ax22.plot(data['abs_time'][:],data['jt_meas'][:,j_ind],'.k', markersize=2, label='jt_meas')
		ax22.set_xlabel('sys. time (sec)', fontsize=15)
		ax22.set_ylabel('jt_meas (Nm)', fontsize=15)

		fig.tight_layout()
		fig.subplots_adjust(hspace=0)

		fig.suptitle('Joint %d' %(j_ind+1), fontsize=16)

		plt.show()

		#==== Diagnotics - Zoomed jp + torque only ===# 
		# fig=plt.figure(figsize=(15,10))	
		# ax11 = fig.add_subplot(2, 1, 1)
		# ax21 = fig.add_subplot(2, 1, 2)

		# ### First joint ####
		# ax11.plot(data['abs_time'][:], data['jp_meas'][:,j_ind],'.k', markersize=5, label='jp_meas')
		# ax11.plot(data['abs_time'][:], data['jp_cmd'][:,j_ind],'.r', markersize=5, label='jp_cmd')
		# ax11.get_xaxis().set_visible(False)
		# ax11.set_ylabel('jp meas vs. cmd (deg.)', fontsize=15)
		# # ax11.legend(loc='best',fontsize=15)
		# # ax11.set_xlim([24,28])
		# ax11.set_xlim([25.56,26.2])
		# ax11.set_ylim([-26.,-20.5])


		# ax21.plot(data['abs_time'][:],data['jt_meas'][:,j_ind],'.k', markersize=5, label='jt_meas')
		# ax21.set_xlabel('sys. time (sec)', fontsize=15)
		# ax21.set_ylabel('jt (Nm)', fontsize=15)
		# ax21.set_xlim([25.56,26.2])

		
		# fig.tight_layout()
		# fig.subplots_adjust(hspace=0)

		# fig.suptitle('Joint %d' %(j_ind+1), fontsize=16)

		# plt.show()


	    # Output
		if not exists(out_dir):
			makedirs(out_dir)

		fig.savefig(join(out_dir,'joint_%d.png' %(j_ind+1)))

if __name__ == '__main__':

	DOF=7
	# datafile='/home/robot/dataLog.csv'
	datafile='../plots/01_03_19/dataLog.csv'
	outpath='../plots/01_03_19/plots/'
	data=load_data(datafile,DOF)
	plot_data(data,outpath,DOF)