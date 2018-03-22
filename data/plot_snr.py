import numpy as np
import matplotlib.pyplot as plt

snr_car_data = np.array([[10.35,20,25.2,26.4,30,37,43.8,55],[7.82153987885,6.55305480957,3.77791762352,4.11192750931,5.00227451324,3.77791762352,4.28322649002,0.410662710667]])
snr_human_data = np.array([[12.49,17.47,22.29,27.5,32.59],[1.44955849648,4.11192750931,0.754210948944,2.01898527145,0.410662710667]])
plt.plot(snr_car_data[0,:],snr_car_data[1,:])
plt.plot(snr_human_data[0,:],snr_human_data[1,:])
plt.grid(which='both')
plt.legend(['car','human'])
plt.axis([0,60,0,10])
plt.title('Autoliv MMR SNR chart')
plt.xlabel('Range (m)')
plt.ylabel('SNR (dB)')
plt.show()