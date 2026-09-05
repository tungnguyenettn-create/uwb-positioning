#!/usr/bin/env python3
import argparse
import csv
import os
import sys
import re

import matplotlib
matplotlib.use("Agg")  # Prevents windows from popping up during batch processing
import matplotlib.pyplot as plt
import numpy as np

# Constants
DT_NS = 1000.0 / (2 * 499.2)
NTM = 13
ALPHA = 6.0
BETA = 0.6
FCN_LOW_FACTOR = 0.6  

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_cir_payload(payload: str):
    """Parses the comma-separated CIR payload (without timestamp)."""
    parts = payload.strip().split(",")
    if parts[0] != "CIR":
        return None

    try:
        raw = {}
        # Support both 14-column and 15-column (with ping_seq) formats
        if len(parts) == 15:
            raw["seq"] = int(parts[1])
            offset = 3
        elif len(parts) == 14:
            raw["seq"] = int(parts[1])
            offset = 2
        else:
            return None

        raw["fp_index_raw"] = int(parts[offset])
        raw["lde_ppindx_raw"] = int(parts[offset+1])
        raw["lde_ppampl"] = int(parts[offset+2])
        raw["fp_ampl1"] = int(parts[offset+3])
        raw["fp_ampl2"] = int(parts[offset+4])
        raw["fp_ampl3"] = int(parts[offset+5])
        raw["std_noise_raw"] = int(parts[offset+6]) # Hardware read - not used for calculation
        raw["rxpacc"] = int(parts[offset+7])
        raw["rx_power"] = float(parts[offset+8])
        raw["fp_power"] = float(parts[offset+9])
        raw["n_samples"] = int(parts[offset+10])
        hex_data = parts[offset+11]
    except ValueError:
        return None

    expected_hex_len = raw["n_samples"] * 4 * 2
    if len(hex_data) != expected_hex_len:
        print(f"[warn] seq {raw['seq']}: hex length {len(hex_data)} != expected {expected_hex_len}", file=sys.stderr)
        return None

    # Treat I and Q as signed 16-bit integers, cast to float64 for math robustness
    iq_bytes = bytes.fromhex(hex_data)
    iq = np.frombuffer(iq_bytes, dtype="<i2").reshape(-1, 2)
    raw["I"] = iq[:, 0].astype(np.float64)
    raw["Q"] = iq[:, 1].astype(np.float64)
    return raw


# ---------------------------------------------------------------------------
# Feature computation
# ---------------------------------------------------------------------------

def compute_features(raw: dict) -> dict:
    I, Q = raw["I"], raw["Q"]
    mag = np.sqrt(I**2 + Q**2)
    N = len(mag)
    n_arr = np.arange(N)

    # 1. Base Locations
    mfp_position = raw["fp_index_raw"] / 64.0
    
    # Bypass hardware calculation and find SP directly from magnitude 
    sp_position = float(np.argmax(mag))
    max_mag = mag[int(sp_position)]

    # 2. Extract Noise region properly
    ens_end = int(np.floor(mfp_position))
    ens_end = max(0, min(ens_end, N))
    ens = mag[:ens_end]
    
    # Calculate noise via the array (bypassing hardware mismatch)
    std_noise = float(np.std(ens)) if len(ens) > 0 else 1.0

    # 3. PNLOS
    def prnlos(idiff):
        if idiff <= 3.3:
            return 0.0
        elif idiff < 6.0:
            return 0.39178 * idiff - 1.31719
        else:
            return 1.0
    pnlos = prnlos(abs(mfp_position - sp_position))

    # 4. Energy and Ratios
    drf = raw["rx_power"] - raw["fp_power"]
    es = max(raw["fp_ampl1"], raw["fp_ampl2"], raw["fp_ampl3"]) / max_mag if max_mag != 0 else float("nan")
    E_R = float(np.sum(mag**2))

    # Setup base variables
    t = n_arr * DT_NS  # Time vector (use 't = n_arr' if you prefer raw sample indices)
    E_R = float(np.sum(mag**2))
    
    # Calculate probability density function psi(t)
    psi = (mag**2) / E_R if E_R > 0 else np.zeros_like(mag)

    # 5) Mean excess delay
    t_med = float(np.sum(t * psi))
    
    # 6) RMS delay spread 
    t_rms = np.sqrt(float(np.sum(((t - t_med)**2) * psi))) 
    
    # 7) Kurtosis 
    mu = np.mean(mag)
    sigma2 = np.mean((mag - mu)**2)
    kurtosis = float(np.mean((mag - mu)**4) / (sigma2**2)) if sigma2 > 0 else float("nan")
   
    # 6. Amplitude difference between F2 and SP
    energy_rise = abs(raw["fp_ampl2"] - std_noise)

    # 7. False Channel Peaks (FCN)
    L_N = float(np.percentile(ens, 90) * 3) if len(ens) > 0 else 0.0
    fcn_thresh = std_noise * FCN_LOW_FACTOR * NTM 

        
    fcn = 0
    if len(ens) >= 3:
        d = np.diff(ens)
        peak_mask = (d[:-1] > 0) & (d[1:] < 0)
        peak_indices = np.where(peak_mask)[0] + 1
        fcn = int(np.sum(ens[peak_indices] > fcn_thresh))

    # 8. True First Path (TFP) Delay
    # Find the first path breaking the L_N Threshold vs MFP
    tfp_indices = np.where(mag > L_N)[0]
    if len(tfp_indices) > 0:
        new_fp = float(tfp_indices[0])
        tfp_delay = (new_fp - mfp_position) * DT_NS
    else:
        tfp_delay = 0.0

    # 9 rise time 
    # Ensure the thresholds are actually crossed to avoid index 0 errors
    # Constants defined in the formula
    ALPHA = 6.0
    BETA = 0.6

    # Calculate thresholds
    thresh_L = ALPHA * std_noise
    thresh_H = BETA * max_mag

    # Define a safe search window strictly around the actual signal
    # Start 30 indices before the First Path (to catch the true leading edge)
    # End 10 indices after the Strongest Path (so we don't scan too far)
    start_idx = max(0, int(mfp_position) - 30)
    end_idx = min(N, int(sp_position) + 10)
    
    window_mag = mag[start_idx:end_idx]
    window_t = t[start_idx:end_idx]

    # Ensure the thresholds are actually crossed within this localized window
    if np.any(window_mag >= thresh_L) and np.any(window_mag >= thresh_H):
        t_L = window_t[np.argmax(window_mag >= thresh_L)]
        t_H = window_t[np.argmax(window_mag >= thresh_H)]
        t_rise = float(t_H - t_L)
        
        # Failsafe: if the noise threshold triggered after the high threshold
        if t_rise< 0:
            t_rise = float("nan")
    else:
        t_rise = float("nan")
    rise_time = t_rise
    return {
        "fcn": fcn,
        "DRF": drf,
        "ES": es,
        "t_rms": t_rms,
        "t_med": t_med,
        "kurtosis": kurtosis,
        "E_R": E_R,
        "PNLOS": pnlos,
        "energy_rise": energy_rise,
        "TFP_Delay": tfp_delay,
        # Plotting helpers (not saved to CSV):
        "_mfp": mfp_position,
        "_sp": sp_position, 
        'rise_time': rise_time 
    }


# ---------------------------------------------------------------------------
# Plotting & Storage
# ---------------------------------------------------------------------------

def plot_cir(raw: dict, features: dict, save_path: str, title: str = None):
    I, Q = raw["I"], raw["Q"]
    mag = np.sqrt(I**2 + Q**2)
    idx = np.arange(len(mag))

    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Plot the main CIR magnitude
    ax.plot(idx, mag, linewidth=1.2, color="tab:blue", label="CIR Magnitude", zorder=2)

    # ---------------------------------------------------------
    # 1. Retrieve Base Features
    # ---------------------------------------------------------
    mfp = features.get("_mfp", 0.0)
    sp = features.get("_sp", float(np.argmax(mag)))
    
    # Re-isolate the noise region to calculate plot thresholds
    ens_end = int(np.floor(mfp))
    ens_end = max(0, min(ens_end, len(mag)))
    ens = mag[:ens_end]
    
    std_noise = float(np.std(ens)) if len(ens) > 0 else 1.0
    
    # ---------------------------------------------------------
    # 2. Calculate Horizontal Thresholds
    # ---------------------------------------------------------
    L_N = float(np.percentile(ens, 90) * 3) if len(ens) > 0 else 0.0
    fcn_thresh = std_noise * 0.6 * 13 
    L = std_noise * 13

    # Plot Horizontal Lines
    ax.axhline(L_N, color="tab:orange", linestyle="--", linewidth=1.5, zorder=3, label=f"L_N = {L_N:.1f}")
    ax.axhline(fcn_thresh, color="tab:purple", linestyle="-.", linewidth=1.5, zorder=3, label=f"FCN Thresh = {fcn_thresh:.1f}")
    ax.axhline(L, color="tab:cyan", linestyle=":", linewidth=1.5, zorder=3, label=f"L = {L:.1f}")

    # ---------------------------------------------------------
    # 3. Calculate Vertical Indices (t_low, t_high)
    # ---------------------------------------------------------
    thresh_L = 6.0 * std_noise
    thresh_H = 0.6 * mag[int(sp)]
    
    # Use the isolated search window
    start_idx = max(0, int(mfp) - 30)
    end_idx = min(len(mag), int(sp) + 10)
    window_mag = mag[start_idx:end_idx]
    
    t_L_idx, t_H_idx = None, None
    if np.any(window_mag >= thresh_L) and np.any(window_mag >= thresh_H):
        t_L_idx = start_idx + np.argmax(window_mag >= thresh_L)
        t_H_idx = start_idx + np.argmax(window_mag >= thresh_H)

    # Plot Vertical Lines
    if 0 <= mfp < len(mag):
        ax.axvline(mfp, color="tab:red", linestyle="-", linewidth=1.5, zorder=4, label=f"F1 (MFP) = {mfp:.1f}")
        
    if 0 <= sp < len(mag):
        ax.axvline(sp, color="tab:green", linestyle="-", linewidth=1.5, zorder=4, label=f"SP = {sp:.0f}")

    if t_L_idx is not None:
        ax.axvline(t_L_idx, color="black", linestyle="--", linewidth=1.2, zorder=4, label=f"t_low = {t_L_idx}")
    if t_H_idx is not None:
        ax.axvline(t_H_idx, color="dimgray", linestyle="--", linewidth=1.2, zorder=4, label=f"t_high = {t_H_idx}")

    # Shade the Rise Time region
    if t_L_idx is not None and t_H_idx is not None and t_H_idx >= t_L_idx:
        ax.axvspan(t_L_idx, t_H_idx, color="gray", alpha=0.3, zorder=1, label=f"t_rise span ({(t_H_idx - t_L_idx)*1.0016:.2f} ns)")

    # ---------------------------------------------------------
    # 4. Formatting and Zooming
    # ---------------------------------------------------------
    # Dynamically zoom around the actual signal (100 samples before F1, 200 after SP)
    zoom_start = max(0, int(mfp) - 100)
    zoom_end = min(len(mag), int(sp) + 200)
    ax.set_xlim(zoom_start, zoom_end)
    
    # Ensure y-axis can fit the highest point or the highest threshold
    ax.set_ylim(0, max(np.max(mag[zoom_start:zoom_end]) * 1.1, L * 1.1))

    ax.set_xlabel("Accumulator Sample Index")
    ax.set_ylabel("Magnitude")
    ax.set_title(title or f"seq={raw.get('seq', 'Unknown')}")
    
    # Move legend outside the plot so it doesn't cover the signal
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=9, borderaxespad=0.)
    ax.grid(True, alpha=0.3)
    
    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

# ONLY processed metadata included, raw dumped into raw file
METADATA_COLUMNS = [
    "timestamp", "seq", "measured_distance", "channel_type", "raw_distance",
    "fcn", "DRF", "ES", "t_rms", "t_med", "kurtosis",
    "E_R", "PNLOS", "energy_rise", "TFP_Delay", "rise_time","npy_file"
]

def append_metadata(csv_path: str, timestamp: str, seq: int, raw_dist: float, features: dict, npy_path: str, distance: str, channel_type:str):
    file_exists = os.path.isfile(csv_path)
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(METADATA_COLUMNS)
            
        row = {
            "timestamp": timestamp, 
            "seq": seq, 
            "raw_distance": raw_dist,
            "npy_file": os.path.basename(npy_path), 
            "measured_distance": distance, 
            "channel_type": channel_type
        }
        
        # Merge in the calculated features
        for key in METADATA_COLUMNS:
            if key in features:
                row[key] = features[key]
                
        writer.writerow([row.get(col, "") for col in METADATA_COLUMNS])

# ---------------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Process a saved DW1000 CIR log file.")
    parser.add_argument("input_file", help="Path to the log file (e.g., distance_LOS.txt)")
    args = parser.parse_args()

    base_name = os.path.splitext(os.path.basename(args.input_file))[0]
    outdir = base_name
    npy_dir = os.path.join(outdir, "raw")
    plot_dir = os.path.join(outdir, "plots")
    csv_path = os.path.join(outdir, f"{base_name}.csv") 

    distance, channel_type = base_name.split('_') 
    distance = float(distance[:-2])/100     

    os.makedirs(npy_dir, exist_ok=True)
    os.makedirs(plot_dir, exist_ok=True)

    print(f"Processing '{args.input_file}' -> Outputting to folder '{outdir}/'...")

    # Regex matches: [timestamp] payload
    log_pattern = re.compile(r'^\[(.*?)\]\s*(.*)$')

    count = 0
    # Store ranges until their matched CIR packet arrives
    ranges_buffer = {}  

    with open(args.input_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            match = log_pattern.match(line)
            if not match:
                continue

            timestamp = match.group(1)
            payload = match.group(2)
            parts = payload.split(",")

            if parts[0] == "RANGE":
                # Assuming format: RANGE, SEQ, RAW_Distance, Rx_POWER
                try:
                    seq = int(parts[1])
                    raw_dist = float(parts[2])
                    ranges_buffer[seq] = raw_dist
                except (ValueError, IndexError):
                    pass

            elif parts[0] == "CIR":
                raw = parse_cir_payload(payload)
                if raw is None:
                    continue

                seq = raw["seq"]
                # Match the CIR with its RANGE raw_distance
                raw_dist = ranges_buffer.pop(seq, float('nan'))

                features = compute_features(raw)

                # Save raw I/Q data
                npy_path = os.path.join(npy_dir, f"{base_name}_{seq:06d}.npy")
                np.save(npy_path, np.stack([raw["I"], raw["Q"]], axis=-1))

                # Save Plot
                png_path = os.path.join(plot_dir, f"{base_name}_{seq:06d}.png")
                plot_cir(raw, features, png_path, title=f"seq={seq} [{timestamp}] | Dist: {raw_dist}")

                # Save Metadata
                append_metadata(csv_path, timestamp, seq, raw_dist, features, npy_path, distance, channel_type)

                count += 1
                if count % 10 == 0:
                    print(f"Processed {count} CIR captures...")

    print(f"\nDone! Successfully processed {count} records into '{outdir}/'")

if __name__ == "__main__":
    main()