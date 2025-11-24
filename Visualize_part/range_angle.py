import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, windows
from scipy.constants import c

NUM_RX, NUM_CHIRP, NUM_SAMPLE = 2, 8, 256
FREQ_START = 58E9
FREQ_STOP = 62E9
BANDWIDTH = FREQ_STOP - FREQ_START
FREQ_CENTER = (FREQ_START + BANDWIDTH/2)
WAVELENGTH = c/FREQ_CENTER

ADC_CLK = 80E6
ADC_DIV = 40
FREQ_SAMPLE = ADC_CLK/ADC_DIV
RAMP_TIME = 70E-6
RAMP_SLOPE = BANDWIDTH/RAMP_TIME

RRES = c/(2 * BANDWIDTH)
RMAX = RRES * NUM_SAMPLE
RBIN = np.arange(0, RMAX, RRES)

SERIAL_PORT = 'COM11'
BAUD_RATE = 921600

#======================================================
#   BINARY SERIAL RECEIVER 
#======================================================
def serial_read(port='COM11', baudrate=921600, n_rx=NUM_RX, n_chirp=NUM_CHIRP, n_sample=NUM_SAMPLE):
    Start_mark = bytes([0xAA, 0x55, 0xAA, 0x55])
    Stop_mark  = bytes([0x55, 0xAA, 0x55, 0xAA])
    Sample = 4

    data_buffer = np.zeros((n_rx, n_chirp, n_sample), dtype=np.uint16)
    frame_count = 0

    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            print(f'Connected to {port} | {baudrate}')

            buffer = bytearray()
            frame_buffer = bytearray()
            in_frame = False
            receive = 0
            total_receive = n_chirp * n_sample

            while True:
                data = ser.read(ser.in_waiting or 1)
                if not data: continue

                buffer.extend(data)
                frame_buffer.extend(data)

                while len(buffer) >= 4:
                    if not in_frame:
                        if buffer[:4] == Start_mark: # FRAME HEAD
                            in_frame = True
                            receive = 0
                            data_buffer.fill(0)
                            frame_buffer = bytearray(buffer[:4])
                            buffer = buffer[4:]
                        else:
                            buffer.pop(0)

                    else:
                        if len(buffer) >= 4 and buffer[:4] == Stop_mark: # FRAME BOTTOM
                            in_frame = False
                            frame_buffer.extend(buffer[:4])
                            buffer = buffer[4:]
                            frame_count += 1
                            
                            yield data_buffer.copy(), frame_count
                            continue

                        if len(buffer) >= Sample:
                            sample_data = buffer[:Sample]
                            buffer = buffer[Sample:]
                            try:
                                value1 = struct.unpack('<H', sample_data[0:2])[0]
                                value2 = struct.unpack('<H', sample_data[2:4])[0]
                                
                                if receive < total_receive:
                                    curr_chirp = receive//n_sample
                                    curr_sample = receive%n_sample
                                    
                                    data_buffer[0, curr_chirp, curr_sample] = value1
                                    data_buffer[1, curr_chirp, curr_sample] = value2
                                    receive += 1
   
                            except struct.error:
                                print('Error : Unable to unpacking data')

    except serial.SerialException as e:
        print(f'Error : {e}')

#======================================================
#   PREPROCESSING RAW ADC DATA
#======================================================
def preproc_adc_data(xn):
    prep = xn - np.mean(xn, axis=2, keepdims=True)
    
    cutoff = 70e3
    b, a = butter(3, cutoff, 'high', fs=FREQ_SAMPLE)
    prep = filtfilt(b, a, prep, axis=2)

    window = windows.nuttall(xn.shape[2])
    prep = prep * window[np.newaxis, np.newaxis, :]

    return prep

#======================================================
#   RANGE ESTIMATE
#======================================================
def range_fft(xn, nfft=512):
    rngs = np.fft.fft(xn, nfft, axis=2)
    return rngs[..., :xn.shape[2]]

#======================================================
#   STATIC BACKGROUND REMOVAL
#======================================================
def removal_background(xn, alpha=0.02, state=None):
    prep = np.mean(xn, axis=0, keepdims=True)
    repica = np.mean(prep, axis=1)
    if state is None:
        bg = {}
    
    if 'background' not in state:
        bg = repica.copy()
        state['background'] = bg
    else:
        bg = state['background']
        state['background'] = (1 - alpha) * bg + alpha * repica

    rev = xn - bg
    return rev, state

#======================================================
#   DOPPLER (VELOCITY) ESTIMATE
#======================================================
def doppler_fft(xn):
    dopplers = np.fft.fft(xn, axis=1)
    dopplers = np.fft.fftshift(dopplers, axes=1)

    return dopplers

#======================================================
#   ANGLE ESTIMATE
#======================================================
def angle_fft(xn):
    azimuths = np.fft.fft(xn, axis=0)
    azimuths = np.fft.fftshift(azimuths, axes=0)

    return azimuths

#======================================================
#   VISUALIZATION
#======================================================
plt.ion()
fig, ax = plt.subplots(1,2, figsize=(12, 5))

line1, = ax[0].plot([], [], 'y-', label='With Static')
line2, = ax[0].plot([], [], 'b-', label='Without Static')
ax[0].set_xlabel('Range (meter)')
ax[0].set_xticks([0,2,4,6,8], [0,1,2,3,4])
ax[0].set_ylim(0, 1000)
ax[0].grid(True)
ax[0].legend(loc='upper right')

angle_bin_dummy = np.linspace(0, 181, 12)
blank_block = np.zeros((len(RBIN), len(angle_bin_dummy)))

range_azimuth_map = ax[1].pcolormesh(angle_bin_dummy, RBIN, blank_block,
                                     shading='auto', cmap='viridis',
                                     vmin=10, vmax=30)
ax[1].set_xlabel('Angle (degree)')
ax[1].set_ylabel('Range (meter)')

plt.tight_layout()

bg_state = {}
frame_counter = 0
reset_frame_intv = 20

def updata_visualize(xn):
    if xn is None: return

    global bg_state, frame_counter
    frame_counter += 1
    if frame_counter % reset_frame_intv == 0:
        bg_state.clear()

    #(1) Preprocessing
    windowed = preproc_adc_data(xn)

    # (2) Range Processing
    rngs = range_fft(windowed)

    # (3) Remove Static background
    rngs_rmv, bg = removal_background(rngs, state=bg_state)
    if bg is not None:
        bg_state.update(bg)

    # (4) Doppler Processing
    doppler = doppler_fft(rngs_rmv)
    doppler = np.pad(doppler, pad_width=[(4,6), (0,0), (0,0)], mode='constant')
    
    # (5) Angle Processing
    azimuth = angle_fft(doppler)

    # (6) Visualize
    rngs_power = np.abs(np.mean(rngs, axis=1))
    line1.set_data(RBIN, rngs_power[0])

    rngs_rmv_power = np.abs(np.mean(rngs_rmv, axis=1))
    line2.set_data(RBIN, rngs_rmv_power[0])

    azimuth_power = np.flipud(np.mean((np.abs(azimuth)), axis=1))
    azimuth_power_log = 10*np.log10(azimuth_power)

    range_azimuth_map.set_array(azimuth_power_log.T)
    ax[1].set_xticks(np.linspace(0, 181, 12), np.linspace(-90, 90, 12).round(1))
    ax[1].set_yticks([0,2,4,6,8], [0,1,2,3,4])

    fig.canvas.draw()
    fig.canvas.flush_events()

if __name__ == '__main__':
    print('-'*50)
    print(f'Number of RX:       {NUM_RX}')
    print(f'Number of Chirp:    {NUM_CHIRP}')
    print(f'Number of Sample:   {NUM_SAMPLE}')
    print(f'Start frequency:    {(FREQ_START*1e-9):.2f} GHz')
    print(f'Stop frequency:     {(FREQ_STOP*1e-9):.2f} GHz')
    print(f'Center Frequency:   {(FREQ_CENTER*1e-9):.2f} GHz')
    print(f'Bandwidth:          {(BANDWIDTH*1e-9):.2f} GHz')
    print(f'Wavelength:         {(WAVELENGTH*1e3):.2f} mm')
    print(f'Ramp End Time:      {(RAMP_TIME*1e6):.2f} usec')
    print(f'Chirp slope:        {(RAMP_SLOPE*1e-12):.2f} MHz/usec')
    print(f'ADC Sample rate:    {(FREQ_SAMPLE*1e-6):.2f} Msps')
    print('-'*50)

    frame = serial_read()
    try: 
        for frame, fidx in frame:
            updata_visualize(frame)

            if fidx % 20 == 0:
                print(f'Frame {fidx}')

    except KeyboardInterrupt:
        print('Interrupt : User Interrupt')
    finally:
        plt.ioff()
        plt.show()
