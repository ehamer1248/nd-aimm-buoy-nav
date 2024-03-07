from queue import Queue, Empty

from navigation import *

def results_consumer(results_queue, stop_event, target_lat, target_lon):
    
    while not stop_event.is_set():
        # Check for new results and process them if available
        try:
            result = results_queue.get(timeout=0.1)
            print(result)
            # process_result(result)  # Process each result to track and adjust based on buoys
        except Empty:
            pass
        
        ###################
        # Actual Control
        ###################
        
        # Update navigation instructions based on the latest GPS and heading
        turn_direction = navigate_to_target(target_lat, target_lon, 50, 50)
        
        # Pass the turn direction to control motors
        control_motors(50, 50, turn_direction)

        # Add a short sleep to prevent this loop from consuming too much CPU
        time.sleep(1)