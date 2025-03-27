
import sys
import time
from pathlib import Path

# Import BlueSky modules
import bluesky as bs

captured_messages = []
num_messages = 0

def print_messages():
    global num_messages
    if len(captured_messages) > num_messages:
        for i in range(num_messages, len(captured_messages)):
            print(captured_messages[i][0])
    num_messages = len(captured_messages)

def my_echo(text='', flags=0):
    captured_messages.append((text, flags))

def test_example_plugin():
    """Run a test for the example plugin in simulation mode."""
    print("Starting BlueSky in simulation mode...")
    
    # Initialize the simulation
    bs.init(mode='sim', detached=True)

    bs.scr.echo = my_echo
    
    # Step 2: Load the example plugin
    bs.stack.stack('PLUGINS LOAD EXAMPLE')
    bs.sim.step()
    print_messages()

    bs.stack.stack('CRE KL123 B744 52.0 4.0 180 FL300 300')
    bs.stack.stack('KL123')
    bs.stack.stack('PASSENGERS KL123')
    bs.sim.step()
    print_messages()

    acid_idx = bs.traf.id.index('KL123') if 'KL123' in bs.traf.id else -1

    if acid_idx == -1:
        print("Error: Aircraft not found in traffic list")
        bs.sim.quit()
        return


    # Clean up
    print("Test complete. Shutting down simulation.")
    bs.sim.quit()

if __name__ == "__main__":
    test_example_plugin()

"""
    # Allow time for plugin to initialize
    bs.sim.step()
    time.sleep(1)
    
    # Step 3: Create one aircraft
    print("Creating an aircraft...")
    result, msg = 
    print(f"Aircraft creation result: {msg}")
    
    # Run a few simulation steps to ensure aircraft is created
    for _ in range(5):
        bs.sim.step()
    
    # Find the aircraft ID in the traffic list
    
    
    
    
    # Step 4: Print the current number of passengers
    print("Getting initial passenger count...")
    result, msg = bs.stack.stack('PASSENGERS KL123')
    print(f"Initial passenger count: {msg}")
    
    # Step 5: Change the number of passengers
    new_passengers = 250
    print(f"Setting passenger count to {new_passengers}...")
    result, msg = bs.stack.stack(f'PASSENGERS KL123 {new_passengers}')
    print(f"Set passengers result: {msg}")
    
    # Run a simulation step
    bs.sim.step()
    
    # Step 6: Print the updated number of passengers
    print("Getting updated passenger count...")
    result, msg = bs.stack.stack('PASSENGERS KL123')
    print(f"Updated passenger count: {msg}")
"""