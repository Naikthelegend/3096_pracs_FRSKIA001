import math

# Number of samples
num_samples = 256

# Generate the sine wave lookup table
sine_wave_table = []
for i in range(num_samples):
    # Calculate the sine value for each sample and scale it to the range 0-1023
    sine_value = int(512 * (1 + math.sin(2 * math.pi * i / num_samples)))
    sine_wave_table.append(sine_value)

# Generate the sawtooth wave lookup table
sawtooth_wave_table = []
for i in range(num_samples):
    # Calculate the sawtooth value for each sample and scale it to the range 0-1023
    sawtooth_value = int(1023 * (i / num_samples))
    sawtooth_wave_table.append(sawtooth_value)

# Generate the triangle wave lookup table
triangle_wave_table = []
for i in range(num_samples//2):
    # Calculate the triangle value for each sample and scale it to the range 0-1023
    triangle_value = int(1023 * (1 - abs((i - num_samples / 2) / (num_samples / 2))))
    triangle_wave_table.append(triangle_value)
for i in range(num_samples//2):
    triangle_wave_table.append(triangle_wave_table[num_samples//2 - i - 1])

flat_wave_table = [512 for i in range(num_samples)];

# Convert the lists to comma-separated strings
sine_wave_csv = ",".join(str(value) for value in sine_wave_table)
sawtooth_wave_csv = ",".join(str(value) for value in sawtooth_wave_table)
triangle_wave_csv = ",".join(str(value) for value in triangle_wave_table)
flat_wave_csv = ",".join(str(value) for value in flat_wave_table)

# Print or use the waveforms CSVs as needed
print("Sine Wave:\n", sine_wave_csv)
print("Sawtooth Wave:\n", sawtooth_wave_csv)
print("Triangle Wave:\n", triangle_wave_csv)
print("Flat Wave:\n", flat_wave_csv);