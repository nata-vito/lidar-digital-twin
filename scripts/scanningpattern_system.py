import numpy as np
import matplotlib.pyplot as plt
import math

# Define the boundary functions
def superior_one(x): 
    return 0.011 * x**2 + 0.63 * x + 20.751

def superior_two(x): 
    return 0.0042 * x**2 + 0.1635 * x + 12.554

def superior_three(x): 
    return 0.0041 * x**2 - 0.1619 * x + 12.696

def superior_four(x): 
    return 0.011 * x**2 - 0.63 * x + 20.751

def inferior_one(x): 
    return 0.1442 * x - 13.073

def inferior_two(x): 
    return 0.1985 * x - 11.235

def inferior_three(x): 
    return -0.2085 * x - 11.016

def inferior_four(x): 
    return -0.1514 * x - 12.599

def right(x): 
    return -17.821 * x - 496.55

def left(x): 
    return 25.623 * x + 1507.1

# j = line index, i = column index & total_lions = total lines
def OFFSET(j,i,total_lions):
    offset =  (j+1) * (inferior_one(i)-superior_one(i))/(total_lions+1)
    
    lidar_start_angle = 25.0/2.0
    horizontal_resolution = 0.1
    lidar_horizontal_fov = 115.0
    pitch = 0
    yaw = 0
    
    lidar_pitch = (pitch + (lidar_start_angle - j * horizontal_resolution)) * math.pi/180
    lidar_yaw = (yaw + (j * horizontal_resolution - (lidar_horizontal_fov/2.0))) * math.pi/180

    end = {
        "X": math.cos(lidar_yaw),
        "Y": math.sin(lidar_yaw),
        "Z": math.sin(lidar_pitch + offset)
    }

    return offset + superior_one(i)


def plot_combined_graph(valid_x_list, top_y_list, bottom_y_list, num_lines):
    plt.figure(figsize=(20, 10))

    colors = ['b', 'g', 'r', 'c']  # Different colors for each region

    valid_x = valid_x_list[0]
    top_y = top_y_list[0]
    bottom_y = bottom_y_list[0]

    # Plotting the superior boundary
    plt.plot(valid_x, top_y, color=colors[0], label=f'Superior Boundary {0+1}')

    # Plotting the inferior boundary
    plt.plot(valid_x, bottom_y, color=colors[0], label=f'Inferior Boundary {0+1}', linestyle='--')

    # Plotting the parallel lines
    for j in range(num_lines):
        plt.plot(valid_x,[OFFSET(j,valid_x[i],num_lines) for i in range(len(valid_x))], color='gray', alpha=0.3)

    # Add labels and legend
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.title('Simulation of Dimensions for the LiDAR Model Projected FOV With 16 lines')
    plt.legend()

    plt.grid(True)
    plt.show()

def main():
    # Define the specific x range where both left and right are defined
    valid_x_one = np.arange(-59.29, -27.92, 0.1) # First region from LiDAR simulation
    valid_x_two = np.arange(-30.1478, 1.9507, 0.1) 
    valid_x_three = np.arange(-1.9507, 30.5025, 0.1)
    valid_x_four = np.arange(27.0738916256157, 59.8817733990147, 0.1)   

    # Calculate corresponding y values for top (superior) and bottom (inferior) boundaries
    top_y_one = superior_one(valid_x_one)
    bottom_y_one = inferior_one(valid_x_one)
    top_y_two = superior_two(valid_x_two)
    bottom_y_two = inferior_two(valid_x_two)
    top_y_three = superior_three(valid_x_three)
    bottom_y_three = inferior_three(valid_x_three)
    top_y_four = superior_four(valid_x_four)
    bottom_y_four = inferior_four(valid_x_four)

    # Number of lines to plot
    num_lines = 256

    plot_combined_graph(
        [valid_x_one, valid_x_two, valid_x_three, valid_x_four],
        [top_y_one, top_y_two, top_y_three, top_y_four],
        [bottom_y_one, bottom_y_two, bottom_y_three, bottom_y_four],
        num_lines
    )


if __name__ == "__main__":
    main()