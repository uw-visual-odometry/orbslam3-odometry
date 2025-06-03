import re
import numpy as np

def extract_values(filename):
    valid_points = []
    size_map_points = []
    elapsed_times = []

    last_lost_count = None
    fail_to_track_count = 0
    creation_of_new_map_count = 0

    with open(filename, 'r') as file:
        for line in file:
            # Extract numeric values
            vp_match = re.search(r'valid_points\s*[:=]\s*(\d+)', line)
            smp_match = re.search(r'Size map point[s]?\s*[:=]\s*(\d+)', line)
            et_match = re.search(r'Elapsed time\s*[:=]\s*([\d.]+)', line)
            lost_match = re.search(r'Lost count\s*[:=]\s*(\d+)', line)

            if vp_match:
                valid_points.append(int(vp_match.group(1)))
            if smp_match:
                size_map_points.append(int(smp_match.group(1)))
            if et_match:
                elapsed_times.append(float(et_match.group(1)))
            if lost_match:
                last_lost_count = int(lost_match.group(1))
            
            # Count messages
            if "Fail to track local map!" in line:
                fail_to_track_count += 1
            if "Creation of new map" in line:
                creation_of_new_map_count += 1

    return (valid_points, size_map_points, elapsed_times,
            last_lost_count, fail_to_track_count, creation_of_new_map_count)

def summarize(values, label):
    if not values:
        print(f"\n--- {label} ---")
        print("No data found.")
        return

    arr = np.array(values)
    print(f"\n--- {label} ---")
    print(f"Count: {len(arr)}")
    print(f"Average: {np.mean(arr):.2f}")
    print(f"Min: {np.min(arr)}")
    print(f"Max: {np.max(arr)}")
    print(f"1st Quartile (25th percentile): {np.percentile(arr, 25):.2f}")
    print(f"3rd Quartile (75th percentile): {np.percentile(arr, 75):.2f}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python analyze_output.py <filename>")
        sys.exit(1)

    filename = sys.argv[1]
    vp, smp, et, lost, fail_count, map_creation_count = extract_values(filename)

    summarize(vp, "Valid Points")
    #summarize(smp, "Size Map Points")
    summarize(et, "Elapsed Time (ms)")

    print(f"\n--- Additional Metrics ---")
    print(f"Final Lost Count: {lost if lost is not None else 'Not found'}")
    print(f"'Fail to track local map!' Count: {fail_count}")
    print(f"'Creation of new map' Count: {map_creation_count}")

