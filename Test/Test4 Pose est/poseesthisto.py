import numpy as np
import matplotlib.pyplot as plt

# Load data from the CSV file
#timestamp, _, _, _, dist_diff, angle_diff = np.loadtxt(r"Pose st\10sec_data_Vicon_reguleret.csv", delimiter=',', unpack=True, skiprows=0)
timestamp, _, _, _, dist_diff, angle_diff = np.loadtxt(r"Pose st\10sec_data.csv", delimiter=',', unpack=True, skiprows=0)

# Round the data
timestamp = np.round(timestamp, 0)
dist_diff = np.round(dist_diff, 2)
angle_diff = np.round(angle_diff, 2)

# Define the bins for the histogram (example: 10s intervals)
bins = np.arange(5, 110, 10)

mean_all = np.mean(dist_diff)
std_all = np.std(dist_diff)
print(f"Uden Vicon; Mean: {mean_all:.2f}, Standard Deviation: {std_all:.2f}")


# Calculate the mean and standard deviation for each bin
mean_values = []
std_dev_values = []
bin_centers = 0.5 * (bins[1:] + bins[:-1])

for i in range(len(bins) - 1):
    bin_mask = (timestamp >= bins[i]) & (timestamp < bins[i+1])
    if np.any(bin_mask):
        bin_values = dist_diff[bin_mask]
        mean_values.append(np.mean(bin_values))
        std_dev_values.append(np.std(bin_values))
    else:
        mean_values.append(0)
        std_dev_values.append(0)

# Create the histogram plot
plt.figure(figsize=(10, 6))
bars = plt.bar(bin_centers, mean_values, width=10, edgecolor='black', alpha=1, label='Distance Error Mean', color='cornflowerblue')

# Add error bars to represent the standard deviation on top of the histogram bars
plt.errorbar(bin_centers, mean_values, yerr=std_dev_values, fmt='o', color='red', label='Standard Deviation', capsize=5)

# Add standard deviation values on top of the error bars
for i, (mean, std_dev) in enumerate(zip(mean_values, std_dev_values)):
    plt.text(bin_centers[i], mean + std_dev, f"{std_dev:.2f}", ha='center', va='bottom', color='black', fontsize=12)

# Set x ticks
plt.xticks(bin_centers, fontsize=14)
#plt.yticks(np.arange(0, 55, 5), fontsize=14)

# Add labels and title
plt.xlabel('Time (s)', fontsize=20)
plt.ylabel('Distance Error Mean [mm]', fontsize=20)
plt.title('Mean Distance Error over time without Vicon correction', fontsize=20)
plt.legend()

# Display the histogram
plt.show()
