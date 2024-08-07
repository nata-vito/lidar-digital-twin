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

def plot_combined_graph(valid_x_list, top_y_list, bottom_y_list, num_lines):
    
    
    lidar_start_angle = 25.0/2.0
    horizontal_resolution = 0.1
    lidar_horizontal_fov = 115.0
    pitch = 0
    yaw = 0
    
        
    plt.figure(figsize=(20, 10))
    ax = plt.axes(projection ="3d")
    colors = ['b', 'g', 'r', 'c']  # Different colors for each region

    for i in range(len(valid_x_list)):
        valid_x = valid_x_list[i]
        top_y = top_y_list[i]
        bottom_y = bottom_y_list[i]

        # Plotting the superior boundary
        plt.plot(valid_x, top_y, color=colors[i], label=f'Superior Boundary {i+1}')

        # Plotting the inferior boundary
        plt.plot(valid_x, bottom_y, color=colors[i], label=f'Inferior Boundary {i+1}', linestyle='--')

        # Plotting the parallel lines
        for j in range(num_lines):
            offset = (j + 1) * (bottom_y - top_y) / (num_lines + 1)
            
            lidar_pitch = (pitch + (lidar_start_angle - j * horizontal_resolution)) * math.pi/180
            lidar_yaw = (yaw + (j * horizontal_resolution - (lidar_horizontal_fov/2.0))) * math.pi/180
            
            end = {
                "X": math.cos(lidar_yaw),
                "Y": math.sin(lidar_yaw),
                "Z": math.sin(lidar_pitch + offset)
            }

            plt.plot(valid_x, top_y + offset, color='gray', alpha=0.3)

    # Add labels and legend
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.title('Simulation of Dimensions for the LiDAR Model Projected FOV With 16 lines')
    plt.legend()

    plt.grid(True)
    plt.show()


def plot_projection():
    # Creating dataset
    z = np.random.randint(100, size =(50))
    x = np.random.randint(80, size =(50))
    y = np.random.randint(60, size =(50))
    
    # Creating figure
    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection ="3d")
    
    # Creating plot
    ax.scatter3D(x, y, z, color = "green")
    plt.title("simple 3D scatter plot")
    
    # show plot
    plt.show()

def regions():
    # Define the specific x range where both left and right are defined
    valid_x_one = np.arange(-59.2906, -27.9015, 0.1)
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

def main():
    plot_projection()

if __name__ == "__main__":
    main()