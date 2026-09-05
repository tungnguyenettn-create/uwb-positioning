import datetime
import sys
import serial
import argparse

def log_serial(port, baud_rate, log_file, max_lines):
    try:
        # Open serial port with a 1-second timeout
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")
        print(
            f"Logging up to {max_lines} lines to {log_file}... Press Ctrl+C to stop early.\n"
        )

        line_count = 0

        with open(log_file, "a", encoding="utf-8") as file:
            while line_count < max_lines:
                if ser.in_waiting > 0:
                    # Read line and decode bytes to text
                    raw_data = ser.readline()
                    line = raw_data.decode("utf-8", errors="replace").rstrip()

                    if line:
                        # Format timestamp (YYYY-MM-DD HH:MM:SS.mmm)
                        timestamp = datetime.datetime.now().strftime(
                            "%Y-%m-%d %H:%M:%S.%f"
                        )[:-3]
                        log_entry = f"[{timestamp}] {line}"

                        # Output to terminal and file
                        print(log_entry)
                        file.write(log_entry + "\n")
                        file.flush()  # Flush buffer immediately to disk

                        line_count += 1

        print(f"\nSuccessfully logged {max_lines} lines. Stopping...")

    except serial.SerialException as e:
        print(f"Serial Port Error: {e}")
        print("Tip: Make sure the port exists and you have permission to access it.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    # Setup argument parser
    parser = argparse.ArgumentParser(description="Log serial data to a text file with timestamps.")
    
    # Define acceptable arguments with fallback defaults
    parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB0", help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("-b", "--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("-f", "--file", type=str, default="test.txt", help="Output log file name")
    parser.add_argument("-m", "--max-lines", type=int, default=500, help="Maximum number of lines to log (default: 500)")

    # Parse arguments from the command line
    args = parser.parse_args()

    # Pass parsed arguments to the logging function
    log_serial(args.port, args.baud, args.file, args.max_lines)