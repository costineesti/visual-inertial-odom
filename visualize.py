import matplotlib.pyplot as plt
import csv

# Initialize lists to store x and y values
values = {
    'data_camera.txt': {'x': [], 'y': []},
    'data_predict.txt': {'x': [], 'y': []},
    'data_update.txt': {'x': [], 'y': []},
}

# Read data from CSV file
for filename, data in values.items():
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Extract x and y values from each row
            data['x'].append(float(row[0]))
            data['y'].append(float(row[1]))

# Function to flip y-values
def flip_y(data):
    return [-y for y in data]

# New axis limits
x_min, x_max = -14.5, 2.5
y_min, y_max = -10, 2.5

# Function to flip and scale y-values ensuring the new points start from [0,0]
def transform_y(data):
    old_min, old_max = min(data), max(data)
    scale_factor = (y_max - y_min) / (old_max - old_min)
    return [(y - data[0]) * scale_factor for y in data]

# Function to transform x-values ensuring the new points start from [0,0]
def transform_x(data):
    old_min, old_max = min(data), max(data)
    scale_factor = (x_max - x_min) / (old_max - old_min)
    return [(x - data[0]) * scale_factor for x in data]

# Plot each dataset individually
plt.figure(figsize=(15, 10))

# Plot for data_predict.txt
plt.subplot(2, 2, 1)
plt.plot(transform_x(values['data_predict.txt']['x']), flip_y(transform_y(values['data_predict.txt']['y'])), linestyle='-', linewidth=4, color='green', label='IMU (Predicție)')
plt.xlabel('Y [metri]')
plt.ylabel('X [metri]')
plt.title('Odometria de la Predicție')
plt.grid(True)
plt.legend()

# Plot for data_camera.txt
plt.subplot(2, 2, 2)
plt.plot(transform_x(values['data_camera.txt']['x']), flip_y(transform_y(values['data_camera.txt']['y'])), linestyle='-', linewidth=4, color='blue', label='Cameră (Update)')
plt.xlabel('Y [metri]')
plt.ylabel('X [metri]')
plt.title('Odometria de la Cameră (Update)')
plt.grid(True)
plt.legend()

# Plot for data_update.txt
plt.subplot(2, 2, 3)
plt.plot(transform_x(values['data_update.txt']['x']), flip_y(transform_y(values['data_update.txt']['y'])), linestyle='-', linewidth=4, color='red', label='Kalman')
plt.xlabel('Y [metri]')
plt.ylabel('X [metri]')
plt.title('Odometria de la filtrul Kalman')
plt.grid(True)
plt.legend()

# Plot for all datasets combined
plt.subplot(2, 2, 4)
for filename, data in values.items():
    if filename == 'data_camera.txt':
        plt.plot(transform_x(data['x']), flip_y(transform_y(data['y'])), linestyle='--', linewidth=4, color='blue', label='Update')  # Dashed line
    elif filename == 'data_predict.txt':
        plt.plot(transform_x(data['x']), flip_y(transform_y(data['y'])), linestyle=':', linewidth=4, color='green', label='Predicție')  # Dotted line
    else:
        plt.plot(transform_x(data['x']), flip_y(transform_y(data['y'])), linestyle='-', linewidth=4, color='red', label='Kalman')  # Solid line

plt.xlabel('Y [metri]')
plt.ylabel('X [metri]')
plt.title('Odometriile pe același grafic')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

