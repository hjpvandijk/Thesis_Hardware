import matplotlib.pyplot as plt
import numpy as np

def draw_ray(start_position, direction, length, color):
    """
    Draws a ray on a 2D plot.

    Args:
        start_position (tuple): A tuple (x, y) representing the starting coordinates of the ray.
        direction (tuple): A tuple (dx, dy) representing the direction vector of the ray.
        length (float): The length of the ray.
    """
    start_x, start_y = start_position
    dx, dy = direction

    # Normalize the direction vector
    magnitude = np.sqrt(dx**2 + dy**2)
    if magnitude == 0:
        raise ValueError("Direction vector cannot have zero magnitude.")
    unit_dx = dx / magnitude
    unit_dy = dy / magnitude

    # Calculate the end position of the ray
    end_x = start_x + length * unit_dx
    end_y = start_y + length * unit_dy

    # Plot the ray
    plt.plot([start_x, end_x], [start_y, end_y], color, label='Ray')
    plt.plot(start_x, start_y, 'bo', label='Start Point') # Mark the start point

    # Add labels and a legend
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Drawing a Ray')
    plt.legend()
    plt.grid(True)
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.gca().set_aspect('equal', adjustable='box') # Ensure equal aspect ratio for proper visualization
    # plt.show()

if __name__ == "__main__":
    # Example usage:
    # start = (1, 2)
    # direction_vector = (3, -1)
    # ray_length = 5

    # try:
    #     draw_ray(start, direction_vector, ray_length)
    # except ValueError as e:
    #     print(f"Error: {e}")
    heading = np.pi/4
    #draw ray in the direction of the heading
    robotpos = (2, 3)
    direction_vector = (np.cos(heading), np.sin(heading))
    ray_length = 1
    draw_ray(robotpos, direction_vector, ray_length, 'p-')


    sensorindex = 2
    angle_in_radians = heading - sensorindex * ( np.pi / 2)  # 90 degrees in radians
    direction_vector2 = (np.cos(angle_in_radians), np.sin(angle_in_radians))
    ray_length2 = 2

    draw_ray(robotpos, direction_vector2, ray_length2, 'b-')

    # sensorindex = 1
    start3 = list(robotpos)  # Convert to list for modification
    if sensorindex == 1 or sensorindex == 3:  # Right or left
        # start position is 0.039 meters (3.9 cm) from the center
        start_pos_opposite = np.sin(heading) * 0.039
        start_pos_adjacent = np.cos(heading) * 0.039
        start3[0] += start_pos_adjacent
        start3[1] += start_pos_opposite

    angle_in_radian3 = heading - sensorindex * ( np.pi / 2)  # 90 degrees in radians
    direction_vector3 = (np.cos(angle_in_radians), np.sin(angle_in_radians))
    ray_length3 = 2

    draw_ray(start3, direction_vector3, ray_length3, 'r-')

    # start3 = (0, 0)
    # direction_vector3 = (-1, -1)
    # ray_length3 = np.sqrt(8) # Example with a non-integer length

    # draw_ray(start3, direction_vector3, ray_length3)

    # # Example of an invalid direction vector
    # start_invalid = (4, 4)
    # direction_invalid = (0, 0)
    # ray_length_invalid = 2
    # try:
    #     draw_ray(start_invalid, direction_invalid, ray_length_invalid)
    # except ValueError as e:
    #     print(f"Error: {e}")
    plt.show()