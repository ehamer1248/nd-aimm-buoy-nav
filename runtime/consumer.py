from queue import Queue, Empty
from navigation import *
from relays import set_relay_state
import time
from 
def results_consumer(results_queue, stop_event):
    key_tracker = 0
    buoy_check = 0
    direction = 0
    buoy_dict = {
        'green': 0,
        'red':   0,
    }
    target_coordinates_dict = {
        'slalom1': [-85.02268076017542, 41.70198248307769],
        'slalom2': [-85.030625496285012, 41.71635825730074],
        'slalom3': [-85.02267890466618, 41.70198396503675],
        'slalom4': [-85.02275610023511, 41.70186289319485],
        'slalom5': [-85.0226950232405, 41.70180377138169],
        'slalom_end': [-85.02266604873908, 41.70179953481259],
        'sensor_turn': [-85.02306737377535, 41.70179157788211],
        'sensor_deployment': [-85.02293789852946, 41.7021783525056],
        'shore_deployment': [-85.0234152136854, 41.70234998178775],
        'search_and_rescue': [-85.02284728578459, 41.70224997588306],
        'finish': [-85.02319139600901, 41.70222611261056],
    }

    target_key_list = [
        'slalom1',
        'slalom2',
        'slalom3',
        'slalom4',
        'slalom5',
        'slalom_end',
        'sensor_turn',
        'sensor_deployment',
        'shore_deployment',
        'search_and_rescue',
        'finish'
    ]

    target_lon = target_coordinates_dict[target_key_list[key_tracker]][0]
    target_lat = target_coordinates_dict[target_key_list[key_tracker]][1]

    loop_count = 0
    total_time = 0

    while not stop_event.is_set():
        start_time = time.time()  # Start measuring time

        # Check for new results and process them if available
        try:
            result = results_queue.get(timeout=0.1)
            buoy_check = 1
            # process_result(result) # Process each result to track and adjust based on buoys
        except Empty:
            buoy_check = 0
            pass

        
        if buoy_check == 1:
            x = result['spatials']['x']
            y = result['spatials']['y']
            z = result['spatials']['z']

            depth = sqrt(sqrt((x**2) + (y**2) + (z**2)))

            if result['class_name'] == 'green_buoy':
                buoy_dict['green'] = depth
            else:
                buoy_dict['red'] = depth

            if abs(buoy_dict['green'] - buoy_dict['red']) < 2:
                control_motors(5,5,'forward')
            elif buoy_dict['green'] > buoy_dict['red']:
                control_motors(5,5,'left')
                time.sleep(2)
                control_motors(5,5,'forward') 
                time.sleep(5)
            elif buoy_dict['green'] < buoy_dict['red']:
                control_motors(5,5,'right')
                time.sleep(2)
                control_motors(5,5,'forward') 
                time.sleep(5)
                
            continue

            

        ###################
        # Actual Control
        ###################
        try:
            current_lat, current_lon, current_heading, current_accHeading = read_gps_data_ublox()

            if current_lat is not None and current_lon is not None:
                # Update navigation instructions based on the latest GPS and heading
                if navigate_to_target(current_lat, current_lon, target_lat, target_lon, 50, 50) is not None:
                    turn_direction = navigate_to_target(current_lat, current_lon, target_lat, target_lon, 50, 50)
                else:
                    key_tracker += 1
                    target_lon = target_coordinates_dict[target_key_list[key_tracker]][0]
                    target_lat = target_coordinates_dict[target_key_list[key_tracker]][1]
                    turn_direction = navigate_to_target(current_lat, current_lon, target_lat, target_lon, 50, 50)

                # Pass the turn direction to control motors
                control_motors(50, 50, turn_direction)
            else:
                print("GPS data not available. Skipping navigation.")
        except Exception as e:
            print(f"An error occurred during navigation: {e}")

        end_time = time.time()  # End measuring time
        loop_time = end_time - start_time  # Calculate loop execution time
        total_time += loop_time
        loop_count += 1

        print(f"Loop {loop_count} execution time: {loop_time:.4f} seconds")

        # Add a short sleep to prevent this loop from consuming too much CPU
        time.sleep(1)

    average_time = total_time / loop_count
    print(f"Average loop execution time: {average_time:.4f} seconds")