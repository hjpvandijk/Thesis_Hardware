import paho.mqtt.client as mqtt
import pygame
import re
import time
import math
import numpy as np
from dataclasses import dataclass
import traceback


# Configuration
MQTT_BROKER_IP = "192.168.1.66"  # Replace with your MQTT broker's IP address
MQTT_TOPIC = "test/topic"
AGENT_ID = "0D351F00F5DF8253"  #  The agent's ID.  The code will filter out messages not for this agent.

# Map and Display settings
PIXELS_PER_METER = 100  # Number of pixels per meter for display
MAP_WIDTH_METERS = 10  # Width of the map in meters
MAP_HEIGHT_METERS = 10  # Height of the map in meters
CELL_SIZE_METERS = 0.312500  # Actual cell size in meters
# CELL_SIZE = int(CELL_SIZE_METERS * PIXELS_PER_METER)  # Size of each cell in pixels (0.31 meters)
GRID_COLOR = (50, 50, 50)
CENTERLINE_COLOR = (255, 255, 0)
OBSTACLE_COLOR = (255, 0, 0)
FREE_COLOR = (0, 255,0)
AGENT_COLOR = (255, 0, 0)
AGENT_PATH_COLOR = (255, 0, 255)  # Color for the agent's path
HEADING_COLOR = (0, 255, 0)
UNKNOWN_COLOR = (128, 128, 128) # Color for unknown cells
TRAIL_LENGTH = 10  # Number of previous positions to keep for the trail

# Global variables
pygame_initialized = False
screen = None
font = None
agent_location = (0.0, 0.0)  # Initial agent position in meters (x, y)
agent_target = (9999, 9999)  # Initial target position in meters (x, y)
agent_heading = 0.0  # Initial agent heading in degrees
agent_path = []
map_data = {}  # (x, y) in cell coordinates, value: 0 (free), 1 (obstacle), None (unknown)
agent_velocities = {}
elapsed_ticks = 0
ticks_per_second = 10
current_time = 0.0

@dataclass
class Box:
    x: float
    y: float
    size: float

# Helper functions
def parse_coordinate(coord_str):
    """Parses a coordinate string "x;y" into a tuple (x, y)."""
    try:
        x, y = map(float, coord_str.split(";"))
        return x, y
    except ValueError:
        return 0.0, 0.0

def vector2_from_string(vector_string):
    """Parses a vector2 string  "x;y" into a tuple (x,y)"""
    try:
        x, y = map(float, vector_string.split(";"))
        return x, y
    except ValueError:
        return 0.0, 0.0

    #             return std::exp(l) / (1 + std::exp(l));
    """Calculates the probability from a log-odds ratio."""
def P(l):
    return math.exp(l) / (1 + math.exp(l))

# double calculatePheromone(double visitedTime, double PConfidence, double currentTime) const {
#             //If the visited time is the same as the current time, the time factor will be 1, so the pheromone will be the same as the sensor confidence.
#             //If the visited time is greater than the current time, an agent's steps went overtime or oscillator drift occurred as a cell cannot have a visited time that is in the future.
#             // So we just take time factor 1 here.
#             if (visitedTime >= currentTime) return PConfidence;
# //            double timeProbability = 1.0 - std::min((currentTime - visitedTime) / EVAPORATION_TIME_S, (1.0 - EVAPORATED_PHEROMONE_FACTOR));
#             double lambda = - std::log(this->EVAPORATED_PHEROMONE_FACTOR) / this->EVAPORATION_TIME_S; //Evaporate to EVAPORATED_PHEROMONE_FACTOR after EVAPORATION_TIME_S
#             double timeProbability = exp(-lambda * (currentTime - visitedTime)); //Exponential decay
#             double pheromone = timeProbability * (PConfidence - 0.5) + 0.5;
#             //This makes sure that a value once set to occupied or free, will not be changed to ambiguous again due to evaporation.
#             //So we assume that if a cell is occupied, it will stay that way, albeit with a lower confidence.
#             //So the asymptote is P_OCCUPIED_THRESHOLD instead of 0.5;
#             if (PConfidence <= P_OCCUPIED_THRESHOLD) pheromone = timeProbability * (PConfidence - P_OCCUPIED_THRESHOLD) + P_OCCUPIED_THRESHOLD;
#             //But if a cell is free, it can become ambiguous again, as new obstacles can appear.
# //            if (PConfidence >= P_FREE) pheromone = timeProbability * (PConfidence - P_FREE) + P_FREE;
#             assert(pheromone >= 0.0 && pheromone <= 1.0 && "Pheromone should be between 0 and 1");
#             return pheromone;
#         }

def calculate_pheromone(visited_time, p_confidence, current_time):
    """
    Calculates the pheromone value based on visited time, confidence, and current time.
    This function is a placeholder and should be replaced with the actual logic.
    """
    # If the visited time is the same as the current time, the time factor will be 1,
    # so the pheromone will be the same as the sensor confidence.
    # If the visited time is greater than the current time, an agent's steps went overtime
    # or oscillator drift occurred as a cell cannot have a visited time that is in the future.
    # So we just take time factor 1 here.
    if visited_time >= current_time:
        return p_confidence

    lambda_ = -math.log(0.5) / 10  # Placeholder for EVAPORATION_TIME_S
    time_probability = math.exp(-lambda_ * (current_time - visited_time))  # Exponential decay
    pheromone = time_probability * (p_confidence - 0.5) + 0.5
    assert pheromone >= 0.0 and pheromone <= 1.0, "Pheromone should be between 0 and 1"
    return pheromone


def quadnode_from_string(node_string):
    """Parses a quadtree node string "x;y:confidence@timestamp"."""
    try:
        parts = node_string.split(":")
        x, y = map(float, parts[0].split(";"))
        confidence_timestamp = parts[1].split("@")
        confidence = P(float(confidence_timestamp[0]))
        timestamp = float(confidence_timestamp[1])
        return x, y, confidence, timestamp
    except ValueError:
        return 0.0, 0.0, 0.0, 0.0

def get_target_id_from_message(message):
    """Extracts the sender ID from the message."""
    match = re.search(r"<(\w+)", message)
    if match:
        return match.group(1)
    return None

def get_id_from_message(message):
    """Extracts the target ID from the message."""
    target_str = message.split('[')[1].split(']')[0]
    return target_str

def split_string(s, delimiter):
    """Splits a string by a delimiter."""
    return s.split(delimiter)

def map_to_display_coordinates(x_m, y_m):
    """
    Converts map coordinates (meters) to display coordinates (pixels).
    Origin (0,0) is assumed to be at the center of the map.
    """
    # x_px = (x_m + (MAP_WIDTH_METERS / 2) - (CELL_SIZE_METERS / 2)) * PIXELS_PER_METER
    # y_px = ((MAP_HEIGHT_METERS / 2) - y_m - (CELL_SIZE_METERS / 2)) * PIXELS_PER_METER

    x_px = (x_m + (MAP_WIDTH_METERS / 2)) * PIXELS_PER_METER
    y_px = ((MAP_HEIGHT_METERS / 2) - y_m ) * PIXELS_PER_METER

    return int(x_px), int(y_px)



def cell_to_display_coordinates(cell_x, cell_y):
    """
    Converts cell coordinates to display coordinates (center of the cell).
    Origin (0,0) is assumed to be at the center of the map.

    Args:
        cell_x: The x-coordinate of the cell (integer).
        cell_y: The y-coordinate of the cell (integer).

    Returns:
        A tuple (x_px, y_px) representing the pixel coordinates of the center
        of the cell on the display.
    """
    # Calculate the center of the cell in meters using the precise cell size.
    cell_x_m = cell_x #* CELL_SIZE_METERS
    cell_y_m = cell_y #* CELL_SIZE_METERS

    # Convert the cell coordinates in meters to pixel coordinates.
    x_px = int((cell_x_m + (MAP_WIDTH_METERS / 2)) * PIXELS_PER_METER)
    y_px = int(((MAP_HEIGHT_METERS / 2) - cell_y_m) * PIXELS_PER_METER)

    # x_px = int((cell_x_m + (MAP_WIDTH_METERS / 2)- (CELL_SIZE_METERS / 2)) * PIXELS_PER_METER)
    # y_px = int(((MAP_HEIGHT_METERS / 2) - cell_y_m - (CELL_SIZE_METERS / 2)) * PIXELS_PER_METER)
    return x_px, y_px

def draw_grid():
    """Draws the grid on the Pygame screen."""
    for x in np.arange(0, MAP_WIDTH_METERS*PIXELS_PER_METER, CELL_SIZE_METERS*PIXELS_PER_METER):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, int(MAP_HEIGHT_METERS*PIXELS_PER_METER)))
    for y in np.arange(0, MAP_HEIGHT_METERS*PIXELS_PER_METER, CELL_SIZE_METERS*PIXELS_PER_METER):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (int(MAP_WIDTH_METERS*PIXELS_PER_METER), y))
    # Draw the center lines
    pygame.draw.line(screen, CENTERLINE_COLOR, (MAP_WIDTH_METERS*PIXELS_PER_METER // 2, 0), (MAP_WIDTH_METERS*PIXELS_PER_METER // 2, int(MAP_HEIGHT_METERS*PIXELS_PER_METER)))
    pygame.draw.line(screen, CENTERLINE_COLOR, (0, MAP_HEIGHT_METERS*PIXELS_PER_METER // 2), (int(MAP_WIDTH_METERS*PIXELS_PER_METER), MAP_HEIGHT_METERS*PIXELS_PER_METER // 2))

    #Draw lines every meter
    for i in range(-int(MAP_WIDTH_METERS/2), int(MAP_WIDTH_METERS/2)+1):
        pygame.draw.line(screen, CENTERLINE_COLOR, (map_to_display_coordinates(i, -int(MAP_HEIGHT_METERS/2))), (map_to_display_coordinates(i, MAP_HEIGHT_METERS)))
        pygame.draw.line(screen, CENTERLINE_COLOR, (map_to_display_coordinates(-int(MAP_WIDTH_METERS/2), i)), (map_to_display_coordinates(MAP_WIDTH_METERS, i)))

    



def draw_map():
    """Draws the map on the Pygame screen based on the map_data."""
    map_data_copy = dict(map_data)
    for (cell_x, cell_y), pheromone in map_data_copy.items():
        for i in range(7):
            s = MAP_WIDTH_METERS / 2**i
            if cell_x % s == 0 and cell_y % s == 0:
                this_cell_size = s * 2
                break

        display_x, display_y = cell_to_display_coordinates(cell_x, cell_y)
        rect = pygame.Rect(
            display_x - int(this_cell_size*PIXELS_PER_METER // 2), display_y - int(this_cell_size*PIXELS_PER_METER) // 2, int(this_cell_size*PIXELS_PER_METER), int(this_cell_size*PIXELS_PER_METER)
        )  # Center the rect
        # print(f"confidence: {confidence}")
        certainty = (pheromone-0.5) * 2
        color = (255*(1-certainty), 255*(certainty), 0) if certainty > 0 else (255*(-certainty), 255*(1+certainty), 0)
        # print(f"Pheromone: {pheromone} -> {color}")
        pygame.draw.rect(screen, color, rect)
        #Draw dark green outline of rectangle
        pygame.draw.rect(screen, (0, 100, 0), rect, 1)
    

def draw_agent(x_m, y_m, x_t, y_t, heading):
    """Draws the agent as a triangle with its heading, using meters."""
    display_x, display_y = map_to_display_coordinates(x_m, y_m)
    point1 = (display_x, display_y)
    angle_rad = math.radians(-heading)
    # print(f"Agent: {x_m}, {y_m} -> {display_x}, {display_y} -> {heading}")
    #0 degrees is pointing to the right, 90 degrees is pointing up.


    point2_x = display_x + 15 * math.cos(angle_rad - math.pi / 6)
    point2_y = display_y + 15 * math.sin(angle_rad - math.pi / 6)
    point3_x = display_x + 15 * math.cos(angle_rad + math.pi / 6)
    point3_y = display_y + 15 * math.sin(angle_rad + math.pi / 6)
    points = [point1, (point2_x, point2_y), (point3_x, point3_y)]
    pygame.draw.polygon(screen, AGENT_COLOR, points)
    if (- MAP_WIDTH_METERS / 2 < x_t < MAP_WIDTH_METERS / 2) and (- MAP_HEIGHT_METERS / 2 < y_t < MAP_HEIGHT_METERS / 2):
        # Draw the target as a small circle
        target_display_x, target_display_y = map_to_display_coordinates(x_t, y_t)
        pygame.draw.circle(screen, (0, 0, 255), (target_display_x, target_display_y), 5)

    heading_x = display_x + 2*PIXELS_PER_METER * math.cos(angle_rad)
    heading_y = display_y + 2*PIXELS_PER_METER * math.sin(angle_rad)
    pygame.draw.line(screen, HEADING_COLOR, (display_x, display_y), (heading_x, heading_y), 2)

def draw_agent_path():
    """Draws the agent's path, which is stored in meters."""
    if len(agent_path) > 1:
        points = [map_to_display_coordinates(x_m, y_m) for x_m, y_m in agent_path]
        pygame.draw.lines(screen, AGENT_PATH_COLOR, False, points, 2)



def on_message(client, userdata, message):
    """Callback function for incoming MQTT messages."""
    global agent_location, agent_target, agent_heading, map_data, elapsed_ticks, agent_velocities, current_time
    payload = message.payload.decode("utf-8")
    try:
        sender_id = get_id_from_message(payload)
        if sender_id != AGENT_ID:
            return

        # target_id = get_target_id_from_message(payload)
        # if target_id != "A" and target_id != AGENT_ID:
        #     return

        message_content = payload.split(']', 1)[1] if ']' in payload else payload

        if message_content.startswith("C:"):
            try:
                parts = message_content[2:].split("|")
                if len(parts) == 2:
                    pos_str, frontier_str = parts
                    x, y = parse_coordinate(pos_str)  # These are in meters now.
                    agent_location = (x, y)  # Update agent location.
                    fx, fy = parse_coordinate(frontier_str)
                    agent_target = (fx, fy)  # Update agent target.
                else:
                    print(f"Error: Expected 2 parts in C message, got {len(parts)}: {message_content}")
            except Exception as e:
                print(f"Error parsing C message: {e}, content: {message_content}")

        elif message_content.startswith("V:"):
            try:
                vector_string = message_content[2:]
                new_vector = vector2_from_string(vector_string)
                agent_velocities[sender_id] = new_vector
            except Exception as e:
                print(f"Error parsing V message: {e}, content: {message_content}")

        elif message_content.startswith("M:"):
            try:
                # map_data.clear()
                chunks = message_content[2:].split("|")
                for chunk in chunks:
                    if chunk:
                        x, y, confidence, timestamp = quadnode_from_string(chunk)
                        current_time = max(current_time, timestamp)
                        print(f"Timestamp: {timestamp}, Confidence: {confidence}")
                        print(f"Current time: {current_time}")
                        # x, y are cell coordinates.
                        # if confidence > 0.5:
                        #     map_data[(x, y)] = 1
                        # elif confidence < -0.5:
                        #     map_data[(x, y)] = 0
                        pheromone = calculate_pheromone(timestamp, confidence, current_time)
                        map_data[(x, y)] = pheromone
            except Exception as e:
                #print stacktrace
                traceback.print_exc()
                print(f"Error parsing M message: {e}, content: {message_content}")
        elif message_content.startswith("T:"):
            pass
        else:
            print(f"Unknown message type: {message_content}")
    except Exception as e:
        print(f"Error processing message: {e}, Payload: {payload}")



def initialize_pygame():
    """Initializes Pygame."""
    global screen, pygame_initialized, font
    pygame.init()
    screen = pygame.display.set_mode((MAP_WIDTH_METERS*PIXELS_PER_METER, MAP_HEIGHT_METERS*PIXELS_PER_METER))
    pygame.display.set_caption("MQTT Map Visualizer")
    font = pygame.font.Font(None, 24)
    pygame_initialized = True

def cleanup_pygame():
    """Cleans up Pygame."""
    if pygame_initialized:
        pygame.quit()

def main():
    """Main function to initialize MQTT client and Pygame."""
    global agent_location, agent_target, agent_heading, agent_path, elapsed_ticks, ticks_per_second, current_time

    client = mqtt.Client()
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER_IP, 1883, 60)
        client.subscribe(MQTT_TOPIC)
        client.loop_start()

        initialize_pygame()

        # map_data[(0.15625, 0.15625)] = 1
        # map_data[(0.3125, 0.3125)] = 1

        running = True
        while running:
            #Check if connected, else reconnect
            if not client.is_connected():
                print("Reconnecting to MQTT broker...")
                client.reconnect()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Simulate agent movement.  agent_location is in meters.
            elapsed_ticks += 1
            if elapsed_ticks % 10 == 0:
                if agent_velocities:
                  first_agent_id = next(iter(agent_velocities))
                  velocity_x, velocity_y = agent_velocities[first_agent_id]
                  agent_heading = math.degrees(math.atan2(velocity_x, velocity_y)) % 360
                #   agent_location = (agent_location[0] + velocity_x * 0.1, agent_location[1] + velocity_y * 0.1) # move by 0.1 meters per second.
                #   agent_heading = (agent_heading + 5) % 360
                  agent_path.append(agent_location)
                  if len(agent_path) > TRAIL_LENGTH:
                      agent_path.pop(0)

            screen.fill((0, 0, 0))
            draw_grid()
            draw_map()
            draw_agent_path()
            draw_agent(*agent_location, *agent_target, agent_heading)
            #draw coordinate (0,0)
            pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(0, 0), 5)
            # for i in range(15):
            #     pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(i*CELL_SIZE_METERS, i*CELL_SIZE_METERS), 5)
            #     pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(-i*CELL_SIZE_METERS, -i*CELL_SIZE_METERS), 5)
            # for i in np.arange(0,5,0.1):
            #     pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(i, i), 5)
            #     pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(-i, -i), 5)
            #draw coordinate (-5,-5)
            # pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(-4, -4), 5)
            # #draw coordinate (5,5)
            # pygame.draw.circle(screen, (255, 255, 0), map_to_display_coordinates(4, 4), 5)

            #publish message
            client.publish(MQTT_TOPIC, f"<A>[1235151]C:{agent_location[0]};{agent_location[1]}|999999999;999999999")

            text_surface = font.render(f"Agent: {AGENT_ID}", True, (255, 255, 255))
            screen.blit(text_surface, (10, 10))

            pygame.display.flip()
            time.sleep(0.1)

    except Exception as e:
        print(f"Error in main loop: {e}")
        traceback.print_exc()
    finally:
        client.loop_stop()
        client.disconnect()
        cleanup_pygame()

if __name__ == "__main__":
    main()
