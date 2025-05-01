import paho.mqtt.client as mqtt
import pygame
import re
import time
import math
import numpy as np
from dataclasses import dataclass
import traceback
import os


# Configuration
# MQTT_BROKER_IP = "192.168.1.66"  # Replace with your MQTT broker's IP address
MQTT_BROKER_IP = "192.168.176.64"
MQTT_TOPIC = "test/topic"
# AGENT_ID = "12F959FD3BDFD31C"  #  The agent's ID.  The code will filter out messages not for this agent.
MAP_AGENT_ID = ""
# Map and Display settings
PIXELS_PER_METER = 100  # Number of pixels per meter for display
MAP_WIDTH_METERS = 9.1  # Width of the map in meters
MAP_HEIGHT_METERS = 9.1 # Height of the map in meters
CELL_SIZE_METERS = 0.284375 #0.15625  # Actual cell size in meters
# CELL_SIZE = int(CELL_SIZE_METERS * PIXELS_PER_METER)  # Size of each cell in pixels (0.31 meters)
GRID_COLOR = (50, 50, 50)
CENTERLINE_COLOR = (255, 255, 0)
OBSTACLE_COLOR = (255, 0, 0)
FREE_COLOR = (0, 255,0)
AGENT_COLOR = (255, 0, 0)
AGENT_PATH_COLOR = (255, 0, 255)  # Color for the agent's path
ROUTE_COLOR = (0, 255, 255)  # Color for the route
HEADING_COLOR = (0, 255, 0)
UNKNOWN_COLOR = (128, 128, 128) # Color for unknown cells
TRAIL_LENGTH = 10  # Number of previous positions to keep for the trail

# Global variables
pygame_initialized = False
screen = None
font = None
agents = {}
LOG_FILE = "mqtt_log.txt"

# agent_location = (0.0, 0.0)  # Initial agent position in meters (x, y)
# agent_target = (9999, 9999)  # Initial target position in meters (x, y)
# agent_heading = 0.0  # Initial agent heading in degrees
# agent_path = []
# map_data = {}  # (x, y) in cell coordinates, value: 0 (free), 1 (obstacle), None (unknown)
route = []  # List of tuples (x, y) in cell coordinates. (start, end)
# agent_velocities = {}
elapsed_ticks = 0
ticks_per_second = 10
# current_time = 0.0

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

    lambda_ = -math.log(0.5) / 9999999  # Placeholder for EVAPORATION_TIME_S
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

    # Draw lines every meter
    for i in range(-int(MAP_WIDTH_METERS / 2), int(MAP_WIDTH_METERS / 2) + 1):
        start_x, start_y = map_to_display_coordinates(i, -MAP_HEIGHT_METERS / 2)
        end_x, end_y = map_to_display_coordinates(i, MAP_HEIGHT_METERS / 2)
        pygame.draw.line(screen, CENTERLINE_COLOR, (start_x, start_y), (end_x, end_y))

        start_x, start_y = map_to_display_coordinates(-MAP_WIDTH_METERS / 2, i)
        end_x, end_y = map_to_display_coordinates(MAP_WIDTH_METERS / 2, i)
        pygame.draw.line(screen, CENTERLINE_COLOR, (start_x, start_y), (end_x, end_y))

    #draw coordinates at ends
    # right_cooiit(text_bottom, (bottom_coordinate))



def find_quadrant(x, y, box_x, box_y, box_size):
    """
    Determines which quadrant (sub-box) the point (x, y) belongs to within the given box.

    Args:
        x: The x-coordinate of the point.
        y: The y-coordinate of the point.
        box_x: The x-coordinate of the center of the box.
        box_y: The y-coordinate of the center of the box.
        box_size: The size (width/height) of the box.

    Returns:
        A tuple (new_box_x, new_box_y) representing the center of the sub-box (quadrant)
        that contains the point (x, y).
    """
    half_size = box_size / 2 #mid to edge

    if x == box_x and y == box_y:
        # Center case
        return box_x, box_y
    
    if x >= box_x and y >= box_y:
        # Top-right quadrant
        return box_x + half_size / 2, box_y + half_size / 2
    elif x < box_x and y >= box_y:
        # Top-left quadrant
        return box_x - half_size / 2, box_y + half_size / 2
    elif x < box_x and y < box_y:
        # Bottom-left quadrant
        return box_x - half_size / 2, box_y - half_size / 2
    elif x >= box_x and y < box_y:
        # Bottom-right quadrant
        return box_x + half_size / 2, box_y - half_size / 2

#Recursive if cell_size > CELL_SIZE_M
def add_to_data_map(x,y, value, box_x, box_y, box_size, sender_id):
    if box_size == CELL_SIZE_METERS:
        agents[sender_id]['map_data'][(box_x,box_y)] = value
    else:
        quadrant = find_quadrant(x, y, box_x, box_y, box_size)
        if quadrant == (box_x, box_y):
            # If the point is in the center, we need to update all quadrants
            # to the same value.
            (top_left_box_x, top_left_box_y) = box_x - box_size / 4, box_y + box_size / 4
            (top_right_box_x, top_right_box_y) = box_x + box_size / 4, box_y + box_size / 4
            (bottom_left_box_x, bottom_left_box_y) = box_x - box_size / 4, box_y - box_size / 4
            (bottom_right_box_x, bottom_right_box_y) = box_x + box_size / 4, box_y - box_size / 4
            add_to_data_map(top_left_box_x, top_left_box_y, value, box_x, box_y, box_size, sender_id)
            add_to_data_map(top_right_box_x, top_right_box_y, value, box_x, box_y, box_size, sender_id)
            add_to_data_map(bottom_left_box_x, bottom_left_box_y, value, box_x, box_y, box_size, sender_id)
            add_to_data_map(bottom_right_box_x, bottom_right_box_y, value, box_x, box_y, box_size, sender_id)
        else:
            new_box_x, new_box_y = quadrant
            new_box_size = box_size / 2
            add_to_data_map(x, y, value, new_box_x, new_box_y, new_box_size, sender_id)
    
def log_message_to_file(message):
    """Logs MQTT messages to a file."""
    try:
        with open(LOG_FILE, "a") as log_file:
            log_file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}.{int(time.time() * 1000) % 1000:03d} - {message}\n")
    except Exception as e:
        print(f"Error logging message to file: {e}")

def draw_map():
    """Draws the map on the Pygame screen based on the map_data."""
    for agent_id, agent in agents.items():
        if agent_id != MAP_AGENT_ID:
            continue
        map_data_copy = dict(agent['map_data'])
        for (cell_x, cell_y), confidence in map_data_copy.items():
            display_x, display_y = cell_to_display_coordinates(cell_x, cell_y)
            rect = pygame.Rect(
                display_x - int(CELL_SIZE_METERS*PIXELS_PER_METER // 2), display_y - int(CELL_SIZE_METERS*PIXELS_PER_METER) // 2, int(CELL_SIZE_METERS*PIXELS_PER_METER), int(CELL_SIZE_METERS*PIXELS_PER_METER)
            )  # Center the rect
            # print(f"confidence: {confidence}")
            pheromone = calculate_pheromone(0, confidence, agent['current_time'])
            # color = (255*(1-certainty), 255*(certainty), 0) if certainty > 0 else (255*(-certainty), 255*(1+certainty), 0)
            color = (255*(1-pheromone), 255*(pheromone), 0)

            # print(f"Pheromone: {pheromone} -> {color}")
            pygame.draw.rect(screen, color, rect)
            #Draw dark green outline of rectangle
            pygame.draw.rect(screen, (0, 100, 0), rect, 1)
    

def draw_agents(agents):
    for agent_id, agent in agents.items():
        """Draws the agent as a triangle with its heading, using meters."""
        x_m, y_m = agent['location']
        x_t, y_t = agent['target']
        heading = agent['heading']
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

        heading_x = display_x + 1.5*PIXELS_PER_METER * math.cos(angle_rad)
        heading_y = display_y + 1.5*PIXELS_PER_METER * math.sin(angle_rad)
        pygame.draw.line(screen, HEADING_COLOR, (display_x, display_y), (heading_x, heading_y), 2)

# def draw_agent_path():
#     """Draws the agent's path, which is stored in meters."""
#     if len(agent_path) > 1:
#         points = [map_to_display_coordinates(x_m, y_m) for x_m, y_m in agent_path]
#         pygame.draw.lines(screen, AGENT_PATH_COLOR, False, points, 2)

def draw_route():
    """Draws the route on the Pygame screen."""
    if len(route) > 1:
        points = [cell_to_display_coordinates(x_m, y_m) for x_m, y_m in route]
        pygame.draw.lines(screen, ROUTE_COLOR, False, points, 2)
        # Draw circles at the start and end of the route
        pygame.draw.circle(screen, ROUTE_COLOR, points[0], 5)
        pygame.draw.circle(screen, ROUTE_COLOR, points[-1], 5)

def on_message(client, userdata, message):
    """Callback function for incoming MQTT messages."""
    global agents, elapsed_ticks, route, MAP_AGENT_ID
    payload = message.payload.decode("utf-8")
    log_message_to_file(payload)
    if payload.startswith("<LOG>"):
        return
    try:        
        sender_id = get_id_from_message(payload)
        if sender_id == 'CLIENT':
            return
        if MAP_AGENT_ID == "":
            MAP_AGENT_ID = sender_id
        if sender_id not in agents:
            agents[sender_id] = {
                'location': (0, 0),
                'target': (0, 0),
                'velocity': (0, 0),
                'heading': 0,
                'traveled_path': 0,
                'battery_usage': 0,
                'current_time' : 0,
                'map_data': {},
            }

        # target_id = get_target_id_from_message(payload)
        # if target_id != "A" and target_id != AGENT_ID:
        #     return

        message_content = payload.split(']', 1)[1] if ']' in payload else payload
        if message_content.startswith("elapsed"):
            return
        if message_content.startswith("C:"):
            try:
                parts = message_content[2:].split("|")
                if len(parts) == 2:
                    pos_str, frontier_str = parts
                    x, y = parse_coordinate(pos_str)  # These are in meters now.
                    # agent_location = (x, y)  # Update agent location.
                    prevlocation = agents[sender_id]['location']
                    traveled_since_last = math.sqrt((x - prevlocation[0])**2 + (y - prevlocation[1])**2)
                    agents[sender_id]['traveled_path'] += traveled_since_last
                    agents[sender_id]['location'] = (x, y)
                    fx, fy = parse_coordinate(frontier_str)
                    # agent_target = (fx, fy)  # Update agent target.
                    agents[sender_id]['target'] = (fx, fy)
                else:
                    print(f"Error: Expected 2 parts in C message, got {len(parts)}: {message_content}")
            except Exception as e:
                print(f"Error parsing C message: {e}, content: {message_content}")

        elif message_content.startswith("V:"):
            try:
                vector_string = message_content[2:]
                new_vector = vector2_from_string(vector_string)
                # agent_velocities[sender_id] = new_vector
                agents[sender_id]['velocity'] = new_vector
                velocity_x, velocity_y = new_vector
                agents[sender_id]['heading'] = math.degrees(math.atan2(velocity_y, velocity_x)) % 360

            except Exception as e:
                print(f"Error parsing V message: {e}, content: {message_content}")

        elif message_content.startswith("M:"):
            # if sender_id != MAP_AGENT_ID:
            #     return
            try:
                # map_data.clear()
                chunks = message_content[2:].split("|")
                for chunk in chunks:
                    if chunk:
                        x, y, confidence, timestamp = quadnode_from_string(chunk)
                        agents[sender_id]['current_time'] = max(agents[sender_id]['current_time'], timestamp)
                        print(f"Timestamp: {timestamp}, Confidence: {confidence}")
                        print(f"Current time: {agents[sender_id]['current_time']}")
                        # x, y are cell coordinates.
                        # if confidence > 0.5:
                        #     map_data[(x, y)] = 1
                        # elif confidence < -0.5:
                        #     map_data[(x, y)] = 0
                        add_to_data_map(x, y, confidence, 0, 0, MAP_WIDTH_METERS, sender_id)
            except Exception as e:
                #print stacktrace
                traceback.print_exc()
                print(f"Error parsing M message: {e}, content: {message_content}")
        elif message_content.startswith("T:"):
            pass
        elif message_content.startswith("R:"):
            print(f"Route message: {message_content}")
            #Draw route
            route.clear()
            chunks = message_content[2:].split("|")
            print(f"Chunks: {chunks}")
            for chunk in chunks:
                if chunk != "":
                    start,end = chunk.split(":")
                    print(f"Start: {start}, End: {end}")
                    start_x, start_y = parse_coordinate(start)
                    end_x, end_y = parse_coordinate(end)
                    route.append((start_x, start_y))
                    route.append((end_x, end_y))


            
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
    pygame.font.init()
    font = pygame.font.SysFont("Arial", 36)  # Font name, size
    pygame_initialized = True

def cleanup_pygame():
    """Cleans up Pygame."""
    if pygame_initialized:
        pygame.quit()

def main():
    """Main function to initialize MQTT client and Pygame."""
    global agents, elapsed_ticks, ticks_per_second, route, LOG_FILE

    client = mqtt.Client()
    client.on_message = on_message

    try:
        LOG_FILE = "mqtt_log.txt"
        #if log file exists, add an int to the filename
        if os.path.exists(LOG_FILE):
            i = 1
            while os.path.exists(f"mqtt_log_{i}.txt"):
                i += 1
            LOG_FILE = f"mqtt_log_{i}.txt"
        client.connect(MQTT_BROKER_IP, 1883, 60)
        client.subscribe(MQTT_TOPIC)
        client.loop_start()

        initialize_pygame()

        # map_data[(0.15625, 0.15625)] = 1
        # map_data[(0.3125, 0.3125)] = 1
        # add_to_data_map(0.625, 0.625, 1, 0, 0, MAP_WIDTH_METERS)
        # add_to_data_map(0.3125, 0.3125, 0.75, 0, 0, MAP_WIDTH_METERS)

        # add_to_data_map(0.15625, 0.15625, 0.5, 0, 0, MAP_WIDTH_METERS)
        # add_to_data_map(0.078125, 0.078125, 0.25, 0, 0, MAP_WIDTH_METERS)
        

        running = True
        i = 0
        while running:
            #Check if connected, else reconnect
            if not client.is_connected():
                print("Reconnecting to MQTT broker...")
                client.reconnect()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Simulate agent movement.  agent_location is in meters.
            # elapsed_ticks += 1
            # if elapsed_ticks % 10 == 0:
            #     if agents:
            #       for agent_id, agent in agents.items():
            #         # first_agent_id = next(iter(agent_velocities))
            #         # velocity_x, velocity_y = agent_velocities[first_agent_id]
            #         #   agent_heading = math.degrees(math.atan2(velocity_y, velocity_x)) % 360
            #         #   print(f"heading: {agent_heading}")
            #             velocity_x, velocity_y = agent['velocity']
            #             agents[agent_id]['heading'] = math.degrees(math.atan2(velocity_y, velocity_x)) % 360
            #         #   agent_location = (agent_location[0] + velocity_x * 0.1, agent_location[1] + velocity_y * 0.1) # move by 0.1 meters per second.
            #         #   agent_heading = (agent_heading + 5) % 360
            #         #   agent_path.append(agent_location)
            #         #   if len(agent_path) > TRAIL_LENGTH:
            #         #       agent_path.pop(0)

            screen.fill((0, 0, 0))
            draw_grid()
            draw_map()
            # map_data.clear()
            # draw_agent_path()
            draw_agents(agents)
            # draw_route()
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

            # pygame.draw.circle(screen, (255, 255, 0), cell_to_display_coordinates(-3.5, 1.4), 5)

            #publish message
            client.publish(MQTT_TOPIC, f"<A>[CLIENT]C:99999;99999|999999999;999999999")

            # text_surface = font.render(f"Agent: {AGENT_ID}", True, (255, 255, 255))
            # screen.blit(text_surface, (10, 10))

            # text_current_time = font.render(f"Current time: {current_time}", True, (255, 255, 255))
            # screen.blit(text_current_time, (10, 50))
            # print current_time for each agent
            for agent_id, agent in agents.items():
                print(f"Agent {agent_id}: {agent['current_time']}")

            if (i%(10*ticks_per_second) == 0):
                #Print traveled path to a file
                with open(f"{LOG_FILE}_traveled_path.txt", "a") as f:
                    for agent_id, agent in agents.items():
                        f.write(f"{agent_id}@{agents[agent_id]['current_time']}: {agent['traveled_path']}\n")
            i += 1
            pygame.display.flip()
            time.sleep(1 / ticks_per_second)

    except Exception as e:
        print(f"Error in main loop: {e}")
        traceback.print_exc()
    finally:
        client.loop_stop()
        client.disconnect()
        cleanup_pygame()

if __name__ == "__main__":
    main()
