#!/usr/bin/env python3
import argparse
import os
import re
import csv

import matplotlib
matplotlib.use("Agg")  # Prevents windows from popping up during batch processing
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description="Extract and plot RX Power and FP Power across samples from a CIR log file.")
    parser.add_argument("input_file", help="Path to the log file (e.g., distance_LOS.txt)")
    args = parser.parse_args()

    # Setup output directory and filenames
    base_name = os.path.splitext(os.path.basename(args.input_file))[0]
    outdir = base_name + "_power_analysis"
    os.makedirs(outdir, exist_ok=True)

    csv_path = os.path.join(outdir, f"{base_name}_powers.csv")
    plot_path = os.path.join(outdir, f"{base_name}_powers.png")

    print(f"Processing '{args.input_file}'...")

    # Regex matches: [timestamp] payload
    log_pattern = re.compile(r'^\[(.*?)\]\s*(.*)$')

    seqs = []
    rx_powers = []
    fp_powers = []

    with open(args.input_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            match = log_pattern.match(line)
            if not match:
                continue

            payload = match.group(2)
            parts = payload.strip().split(",")

            if parts[0] == "CIR":
                try:
                    # Support both 14-column and 15-column (with ping_seq) formats
                    # By checking your original parse offset, rx_power and fp_power are at these indices
                    if len(parts) == 15:
                        seq = int(parts[1])
                        rx_power = float(parts[11])
                        fp_power = float(parts[12])
                    elif len(parts) == 14:
                        seq = int(parts[1])
                        rx_power = float(parts[10])
                        fp_power = float(parts[11])
                    else:
                        continue
                    
                    seqs.append(seq)
                    rx_powers.append(rx_power)
                    fp_powers.append(fp_power)
                except ValueError:
                    continue

    if not seqs:
        print("No valid CIR power data found in the file.")
        return

    # 1. Save to CSV
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["seq", "rx_power", "fp_power"])
        for s, rx, fp in zip(seqs, rx_powers, fp_powers):
            writer.writerow([s, rx, fp])
    print(f"Extracted {len(seqs)} records.")
    print(f"Data saved to {csv_path}")

    # 2. Plot the data
    plt.figure(figsize=(12, 6))
    
    # Plot RX Power (raw_power) and FP Power
    plt.plot(seqs, rx_powers, label='RX Power', color='tab:blue', alpha=0.8, linewidth=1.5)
    plt.plot(seqs, fp_powers, label='FP Power', color='tab:orange', alpha=0.8, linewidth=1.5)
    
    # Fill the gap between the two to visualize the DRF (Difference in Receive/First-path)
    plt.fill_between(seqs, rx_powers, fp_powers, color='gray', alpha=0.1, label='Power Difference (DRF)')

    plt.title(f"RX Power vs FP Power over Samples\nFile: {base_name}")
    plt.xlabel("Sample Sequence Number (seq)")
    plt.ylabel("Power (dBm)")
    plt.legend(loc="upper right")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    
    plt.savefig(plot_path, dpi=150)
    plt.close()
    
    print(f"Plot saved to {plot_path}")
    print("Done!")

if __name__ == "__main__":
    main()