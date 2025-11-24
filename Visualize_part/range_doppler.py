import serial
import numpy as np
import struct
import matplotlib.pyplot as plt
import scipy.fft
import scipy.signal
from scipy.constants import c
from scipy import ndimage

#@ SERIAL READ
PORT = 'COM11'
BAUD = 921600

NUM_RX = 1
NUM_CHIRP = 64
NUM_SAMPLE = 256

def rserial(port=PORT,baud=BAUD,rx=NUM_RX,chirp=NUM_CHIRP,sample=NUM_SAMPLE):
    START_MARKER = bytes([0xAA, 0x55, 0xAA, 0x55])  
    STOP_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
    SAMPLE_SIZE = 2

    bin_data = np.zeros((rx, chirp, sample), dtype=np.uint16)
    frame_count = 0

    try:
        with serial.Serial(port,baud,timeout=2) as ser:
            print(f'CONNECTED {port} - {baud}')

            buffer = bytearray()
            inframe = False
            sample_receive = 0
            total_sample = chirp * sample
            frame_data = bytearray() 

            while True:
                data = ser.read(ser.in_waiting or 1)
                if not data:    
                    continue

                buffer.extend(data)
                frame_data.extend(data)

                while len(buffer) >= 4:
                    if not inframe:
                        if buffer[:4] == START_MARKER:
                            inframe = True
                            sample_receive= 0
                            bin_data.fill(0)
                            frame_data = bytearray(buffer[:4])
                            buffer = buffer[4:]
                            #print('Frame Start')
                        else:
                            buffer.pop(0)
                    else:
                        if len(buffer) >= 4 and buffer[:4] == STOP_MARKER:
                            inframe = False
                            frame_data.extend(buffer[:4])
                            buffer = buffer[4:]
                            frame_count += 1
                            #print(f'Frame {frame_count} Complete - {sample_receive} samples')
                            yield bin_data.copy(), frame_count
                            continue

                        if len(buffer) >= SAMPLE_SIZE:
                            sample_data = buffer[:SAMPLE_SIZE]
                            buffer = buffer[SAMPLE_SIZE:]
                            try:
                                values = struct.unpack('<H', sample_data)[0]
                                if sample_receive < total_sample:
                                    chirp_idx = sample_receive//sample
                                    sample_idx = sample_receive % sample
                                    bin_data[0, chirp_idx, sample_idx] = values
                                    sample_receive += 1
                                    if sample_receive >= total_sample:
                                        print('All sample received')
                            except struct.error:
                                print('ERROR upacking data')
                                continue

    except serial.SerialException as e:
        print(f'SERIAL ERROR : {e}')
    except KeyboardInterrupt:
        print('STOP')


#@ PARAMETER
FC = 60e9
BW = 4e9
TC = 134e-6
SL = BW/TC
WL = c/FC

FCLK = 80e6
ADC_DIV = 40
FS = FCLK/ADC_DIV

FRES = FS/NUM_SAMPLE
RMAX = (FS*c)/(2*SL)
RRES = c/(2*BW)
VMAX = WL/(4*TC)
VRES = WL/(2*NUM_CHIRP*TC)

#@ PREPROCESSING
def preproc_signal(xn): # <- (R,C,S)
    prep = xn - np.mean(xn, axis=2, keepdims=True)
    #prep = xn

    cutoff = 80e3
    b, a = scipy.signal.butter(3, cutoff, btype='high', fs=FS)
    scipy.signal.filtfilt(b, a, prep, axis=2)

    window = scipy.signal.windows.blackman(NUM_SAMPLE)
    prep = prep * window[np.newaxis, np.newaxis, :]

    return prep # -> (R,C,S)

#@ RANGE PROCESSING
def range_fft(xn, nfft=512): # <- (R,C,S)
    proc = scipy.fft.fft(xn, nfft, axis=2)
    #proc = proc / (nfft/2)
    rbin = scipy.fft.fftfreq(nfft, 1/FS) * (c/(2*SL))

    return proc[..., :NUM_SAMPLE], rbin[:NUM_SAMPLE] # -> (R,C,S)

#@ BACKGROUND SUBTRACT
def static_subtract(xn, ema_alp=0.02, bg_state=None): # <- (R,C,S)
    repetition = np.mean(xn, axis=1)
    if bg_state is None:
        bg = {}
    if 'background' not in bg_state:
        bg = repetition.copy()
        bg_state['background'] = bg
    else:
        bg = bg_state['background']
        bg_state['background'] = (1 - ema_alp) * bg + ema_alp * repetition

    residual = xn - bg[:, np.newaxis, :]
    
    return residual, bg_state # -> (R,C,S)

#@ DOPPLER PROCESSING
def doppler_fft(xn): # <- (R,C,S)
    window = scipy.signal.windows.blackman(xn.shape[1])
    proc = xn * window[np.newaxis, :, np.newaxis]
    proc = scipy.fft.fft(xn, axis=1)
    proc = scipy.fft.fftshift(proc, axes=1)

    return proc # -> (R,C,S)

#@ 1D CFAR across Range
NUM_REF = 12
NUM_GUARD = 8
BIAS = 1

def ca_cfar1(xn, r=NUM_REF, g=NUM_GUARD, b=BIAS, method='average'): # <- (1,256)
    rx, sample = xn.shape
    frame = np.mean(xn, axis=0)

    cfar_values = np.zeros_like(frame) 
    targets_only = np.zeros_like(frame)

    for center_idx in range(g + r, sample - (g + r)): 
        min_idx = center_idx - (g + r)
        max_idx = center_idx + (g + r) + 1

        min_guard = center_idx - g
        max_guard = center_idx + g + 1

        lower_nearby = frame[min_idx:min_guard]
        upper_nearby = frame[max_guard:max_idx]

        lower_mean = np.mean(lower_nearby)
        upper_mean = np.mean(upper_nearby)

        if method == 'average':  
            mean = np.mean(np.concatenate((lower_nearby, upper_nearby)))
        elif method == 'greatest':
            mean = max(lower_mean, upper_mean)
        elif method == 'smallest':
            mean = min(lower_mean, upper_mean)
        else:
            mean = 0

        output = mean * b
        #print(f'mean   : {mean}')
        #print(f'output : {output}')
        cfar_values[center_idx] = output

        if frame[center_idx] > output:
            targets_only[center_idx] = frame[center_idx] # <- out of index 18 ?

    return cfar_values, targets_only # -> (S) , (S)

#@ 2D CFAR aross Range-Doppler
NUM_REF2 = 4
NUM_GUARD2 = 4
BIAS2 = 1.2

def ca_cfar2(xn, r=NUM_REF2, g=NUM_GUARD2, b=BIAS2): # <- (C,S)
    chirp, sample = xn.shape
    cfar_values = np.zeros_like(xn)
    targets_only = np.zeros_like(xn)

    for row_idx in range(g + r, chirp - g + r):
        for col_idx in range(g + r, sample - g + r):
            # row side
            min_idx_row = row_idx - (g + r)
            max_idx_row = row_idx + (g + r) + 1

            min_guard_row = row_idx - g
            max_guard_row = row_idx + g + 1
    
            # col side
            min_idx_col = col_idx - (g + r)
            max_idx_col = col_idx + (g + r) + 1

            min_guard_col = col_idx - g
            max_guard_col = col_idx + g + 1
    
            lower_nearby = xn[min_idx_row:min_guard_row, min_idx_col:min_guard_col] # -> shape : (4, 4)
            upper_nearby = xn[max_guard_row:max_idx_row, max_guard_col:max_idx_col] # -> shape : (4, 4)

            mean = np.mean(np.concatenate((
                lower_nearby.flatten(), upper_nearby.flatten()
                )))

            output = mean * b
            cfar_values[row_idx, col_idx] = output

            if xn[row_idx, col_idx] > output:
                targets_only[row_idx, col_idx] = xn[row_idx, col_idx]


    return cfar_values, targets_only # -> (C,S)


#@ VISUALIZATION
plt.ion()
fig, ax = plt.subplots(2,2, figsize=(12,8))

rngs_line, = ax[0,0].plot([],[],'y-',label='Range Spectrum')
stout_line, = ax[0,0].plot([],[],'m-',label='Static Removal')
ax[0,0].set_xlabel('range (m)')
ax[0,0].grid(True)
ax[0,0].legend(loc='upper right')

rngs2_line, = ax[1,0].plot([],[],'c-',label='Range Spectrum')
stout2_line, = ax[1,0].plot([],[],'r-',label='Static Removal')
ax[1,0].set_xlabel('range (m)')
ax[1,0].grid(True)
ax[1,0].legend(loc='upper right')

rbin = np.linspace(0, RMAX/2, NUM_SAMPLE)
vbin = np.linspace(-VMAX/2, VMAX/2, NUM_CHIRP)
blank = np.zeros((len(rbin), len(vbin)))

rdmap = ax[0,1].pcolormesh(vbin, rbin, blank, shading='auto', cmap='viridis', vmin=0, vmax=80)
ax[0,1].set_xlabel('velocity (m/s)')
ax[0,1].set_ylabel('range (m)')
ax[0,1].grid(True)

scatter_point, = ax[1,1].plot([], [], c='r', marker='o')
cfarmap = ax[1,1].pcolormesh(vbin, rbin, blank, shading='auto', cmap='viridis', vmin=1000, vmax=2500)
ax[1,1].set_xlabel('velocity (m/s)')
ax[1,1].set_ylabel('range (m)')
ax[1,1].grid(True)

plt.tight_layout()

bg_state = {}
frame_counter = 0
reset_intv = 10

def update_plot(xn):
    if xn is None:  return

    # ---- Range Visualize ----
    windowed = preproc_signal(xn)               # -> (R,C,S)
    rframe, rngs_bin = range_fft(windowed)      # -> (R,C,S)
    rframe_sum = np.abs(rframe.sum(axis=1))     # -> (R,S)
    rngs_line.set_data(rngs_bin, rframe_sum)
    ax[0,0].set_xlim(0, np.max(rngs_bin))
    ax[0,0].set_ylim(0, 30000)

    rframe_log = 20 * np.log10(rframe_sum + 1e-12)
    rngs2_line.set_data(rngs_bin, rframe_log)
    ax[1,0].set_xlim(0, np.max(rngs_bin))
    ax[1,0].set_ylim(0, 120)

    # ---- BG Removal Visualize ----
    global bg_state, frame_counter
    frame_counter += 1
    if frame_counter % reset_intv == 0: 
        bg_state.clear()

    srframe, sr_bg = static_subtract(rframe, ema_alp=0.02, bg_state=bg_state)
    if sr_bg is not None: 
        bg_state.update(sr_bg)

    srframe_sum = np.abs(srframe.sum(axis=1))
    stout_line.set_data(rngs_bin, srframe_sum)
    ax[0,0].set_xlim(0, np.max(rngs_bin))
    ax[0,0].set_ylim(0, 30000)

    srframe_log = 20 * np.log10(srframe_sum + 1e-12)
    stout2_line.set_data(rngs_bin, srframe_log)
    ax[1,0].set_xlim(0, np.max(rngs_bin))
    ax[1,0].set_ylim(0, 120)

    # ---- Doppler Visualize ----
    dmap = doppler_fft(srframe)
    dmap_mag = np.abs(dmap)
    dmap_sum = dmap_mag.sum(axis=0)
    dmap_sum = 20 * np.log10(dmap_sum + 1e-12) # (C,S)

    rdmap.set_array(dmap_sum.T)

    # ---- 2D CFAR ----
    dmap = doppler_fft(srframe)
    dmap_mag = np.abs(dmap)
    dmap_sum = dmap_mag.sum(axis=0)

    _ , target2 = ca_cfar2(dmap_sum)
    cfarmap.set_array(target2.T)

    local_max = (dmap_sum == ndimage.maximum_filter(dmap_sum, size=(3,3)))
    cand_mask = local_max & (target2 > 0) 

    rows, cols = np.where(cand_mask)
    groups = []
    for r, c in zip(rows, cols):
        range_m = rngs_bin[c]
        vel_m_s = vbin[r]
        peak_lin = dmap_sum[r, c]
        peak_db  = 20*np.log10(peak_lin + 1e-12)
        
        if peak_db >= 68:
            groups.append({
                "range_m": float(range_m),
                "velocity_m_s": float(vel_m_s),
                "peak_linear": float(peak_lin),
                "peak_db": float(peak_db)
            })

    groups = sorted(groups, key=lambda g: g['peak_db'], reverse=True)[:4]

    print(len(groups))
    
    for idx, g in enumerate(groups, start=1):
        print(f"Target{idx} : {g['range_m']:.2f} m | {g['velocity_m_s']:.2f} m/s | {g['peak_db']} dB")

    # -----------------

    fig.canvas.draw()
    fig.canvas.flush_events()


if __name__ == "__main__":
    frame = rserial()
    try:
        for frame, fidx in frame:
            print(f'Processing {fidx}')

            update_plot(frame)
    except KeyboardInterrupt:
        print('STOP')
    finally:
        plt.ioff()
        plt.show()