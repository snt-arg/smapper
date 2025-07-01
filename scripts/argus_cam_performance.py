import time
import psutil
from statistics import mean
from jtop import jtop


def find_pid_by_name(name):
    """Find PID for the first process with a given name."""
    for proc in psutil.process_iter(attrs=['pid', 'name']):
        if name in proc.info['name']:
            return proc.info['pid']
    return None


def monitor_process(pid, duration=60):
    cpu_percent_samples = []
    ram_usage_samples = []
    gpu_mem_samples = []

    with jtop() as jetson:
        print(f"Monitoring PID {pid} for {duration} seconds...")
        process = psutil.Process(pid)

        for _ in range(duration):
            if not jetson.ok():
                break

            # CPU%
            try:
                cpu = process.cpu_percent(interval=0.1)
                ram = process.memory_info().rss / (1024 ** 2)  # in MB
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                print("Process not found or access denied.")
                break

            gpu_mem = jetson.stats['GPU']

            cpu_percent_samples.append(cpu)
            ram_usage_samples.append(ram)
            gpu_mem_samples.append(gpu_mem)

            time.sleep(1)

    return {
        "CPU%": mean(cpu_percent_samples) if cpu_percent_samples else 0,
        "RAM MB": mean(ram_usage_samples) if ram_usage_samples else 0,
        "GPU MEM MB": mean(gpu_mem_samples) if gpu_mem_samples else 0
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Monitor Jetson process usage averages.")
    parser.add_argument("process", help="Process name or PID to monitor")
    parser.add_argument("--duration", type=int, default=60, help="Duration in seconds to monitor")

    args = parser.parse_args()

    # Determine PID
    pid = int(args.process) if args.process.isdigit() else find_pid_by_name(args.process)
    if pid is None:
        print(f"Process '{args.process}' not found.")
        exit(1)

    averages = monitor_process(pid, duration=args.duration)

    print("\n=== Averages over {} seconds ===".format(args.duration))
    for key, val in averages.items():
        print(f"{key}: {val:.2f}")
