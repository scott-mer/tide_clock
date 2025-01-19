import uasyncio as asyncio
from machine import Pin
import ntptime
import network
import time
import urequests
import ujson

# Load Wi-Fi configuration from config.json
def load_config(file_path="config.json"):
    with open(file_path, 'r') as f:
        return ujson.load(f)

# Load credentials
config = load_config()
SSID = config["SSID"]
PASSWORD = config["PASSWORD"]
STATION_ID = config["STATION_ID"]

# NOAA API URL
NOAA_API_URL = (
    f"https://api.tidesandcurrents.noaa.gov/api/prod/datagetter"
    f"?date=today&station={STATION_ID}&product=predictions&datum=STND"
    f"&time_zone=gmt&units=english&format=json"
)

# Globals
TIME = ""
LAST_HOME_TIME = "Unknown"
TIDE_DATA = None
CURRENT_TIDE_VALUE = None  # <--- Stores the closest current tide value
motor = None  # Initialize motor as global
TIDE_TRACKING_MODE = False  # Toggle tide tracking; not reset on page loads

# Updated HTML content for the web page
HTML = """\
<!DOCTYPE html>
<html>
<head>
    <title>Tide Clock</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; }
        h1 { color: #333; }
        p { color: #666; }
        button { padding: 10px 20px; font-size: 16px; margin: 10px; cursor: pointer; }
        input { padding: 5px; font-size: 14px; width: 70px; margin: 10px; text-align: center; }
    </style>
    <script>
        async function home() {
            try {
                const response = await fetch('/home');
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                const state = await response.text();
                document.getElementById('home-state').innerText = "Last Homed: " + state;
            } catch (error) {
                console.error('Error:', error);
                document.getElementById('home-state').innerText = "Error homing.";
            }
        }

        // Single call to retrieve both tide level and stepper position
        async function updateStatus() {
            try {
                const response = await fetch('/status');
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                const data = await response.json();
                document.getElementById('tide-level').innerText = "Tide Level: " + data.tide;
                document.getElementById('home-state').innerText = "Last Homed: " + data.lastHomedTime;
                document.getElementById('tide-tracking-status').innerText = "Tide Tracking: " + (data.tideTracking ? "ON" : "OFF");
                document.getElementById('stepper-position').innerText = "Stepper Position: " + data.position;
            } catch (error) {
                console.error('Error:', error);
            }
        }

        async function moveStepper() {
            const steps = parseInt(document.getElementById('steps-input').value) || 0;
            try {
                const response = await fetch(`/move?steps=${steps}`);
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                // After moving, refresh status
                updateStatus();
            } catch (error) {
                console.error('Error:', error);
            }
        }

        async function setMaxPosition() {
            const newMax = parseInt(document.getElementById('maxpos-input').value) || 0;
            try {
                const response = await fetch(`/maxpos?value=${newMax}`);
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                // After updating, refresh status
                updateStatus();
            } catch (error) {
                console.error('Error:', error);
            }
        }
        
        async function toggleTideTracking() {
            try {
                const response = await fetch('/toggle_tide');
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                const data = await response.json();
                document.getElementById('tide-tracking-status').innerText = "Tide Tracking: " + (data.enabled ? "ON" : "OFF");
            } catch (error) {
                console.error('Error toggling tide tracking:', error);
            }
        }

        document.addEventListener('DOMContentLoaded', () => {
            updateStatus();
            // Update tide+position periodically (e.g. every 0.5s). Adjust as needed.
            setInterval(updateStatus, 500);
        });
    </script>
</head>
<body>
    <h1>Tide Clock Dashboard</h1>
    <p id="tide-level">Tide Level: Unknown</p>
    
    <h2>Stepper Motor Control</h2>
    <p id="home-state">Last Homed: Unknown</p>
    <button onclick="home()">Home</button>
    
    <p id="stepper-position">Stepper Position: Unknown</p>
    <div>
      <label for="steps-input">Steps to move:</label>
      <input type="number" id="steps-input" value="10">
      <button onclick="moveStepper()">Move Stepper</button>
    </div>

    <div>
      <label for="maxpos-input">Max Position:</label>
      <input type="number" id="maxpos-input" value="850">
      <button onclick="setMaxPosition()">Set Max Position</button>
    </div>
    
    <h2>Tide Tracking</h2>
    <button onclick="toggleTideTracking()">Toggle Tide Tracking</button>
    <p id="tide-tracking-status">Tide Tracking: OFF</p>
    
</body>
</html>
"""

async def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print(f"Connecting to Wi-Fi network: {SSID}")
        try:
            wlan.connect(SSID, PASSWORD)
        except Exception as e:
            print(f"Failed to initiate Wi-Fi connection: {e}")
            return None

        timeout = 30  # seconds
        start_time = time.time()
        while not wlan.isconnected():
            if time.time() - start_time > timeout:
                print("Wi-Fi connection timeout.")
                return None
            print("Waiting for connection...")
            await asyncio.sleep(1)

    print("Wi-Fi connected!")
    print("IP Address:", wlan.ifconfig()[0])
    return wlan

async def fetch_time():
    global TIME
    try:
        print("Fetching time from NTP server...")
        ntptime.host = 'pool.ntp.org'  # Optional: Use a specific NTP server
        ntptime.settime()
        # Convert to local time if needed
        current_time = time.localtime()
        TIME = f"{current_time[0]:04}-{current_time[1]:02}-{current_time[2]:02} {current_time[3]:02}:{current_time[4]:02}:{current_time[5]:02}"
        print(f"Time updated: {TIME}")
    except Exception as e:
        print(f"Failed to fetch time: {e}")

def fetch_json_data():
    global TIDE_DATA
    try:
        print("Fetching JSON data from NOAA API...")
        response = urequests.get(NOAA_API_URL)
        if response.status_code != 200:
            print(f"NOAA API returned status code {response.status_code}")
            response.close()
            return

        raw_data = response.json()
        response.close()

        # Normalize data
        values = [float(item['v']) for item in raw_data.get('predictions', []) if 'v' in item]
        if not values:
            print("No tide predictions found in the data.")
            return

        min_val, max_val = min(values), max(values)
        if min_val == max_val:
            print("Tide data has no variation.")
            return

        TIDE_DATA = []
        for item in raw_data.get('predictions', []):
            if 't' in item and 'v' in item:
                val = float(item['v'])
                normalized = (val - min_val) / (max_val - min_val)
                TIDE_DATA.append({'t': item['t'], 'v': normalized})

        print("Data fetched and normalized.")
    except Exception as e:
        print(f"Failed to fetch or process data: {e}")

# ------------------------
# NEW TIME-PARSING HELPERS
# ------------------------
def parse_time_str(t_str):
    """
    Parses a NOAA timestamp of form 'YYYY-MM-DD HH:MM'
    into a (year, month, day, hour, minute, second, wday, yday) tuple.
    We set second=0, wday=0, yday=0. You can adjust as needed.
    """
    date_part, time_part = t_str.split(' ')
    year, month, day = [int(x) for x in date_part.split('-')]
    hour, minute = [int(x) for x in time_part.split(':')]
    return (year, month, day, hour, minute, 0, 0, 0)

def parse_time_str_with_seconds(t_str):
    """
    Parses TIME string 'YYYY-MM-DD HH:MM:SS' into a tuple
    that you can pass to time.mktime (if supported).
    """
    date_part, time_part = t_str.split(' ')
    year, month, day = [int(x) for x in date_part.split('-')]
    hour, minute, second = [int(x) for x in time_part.split(':')]
    return (year, month, day, hour, minute, second, 0, 0)

async def update_tide_level():
    """
    Periodically picks the closest TIDE_DATA entry to the current TIME.
    If TIDE_TRACKING_MODE is enabled, moves the stepper to that position.
    """
    global TIDE_TRACKING_MODE, motor, CURRENT_TIDE_VALUE

    while True:
        try:
            print("DEBUG: update_tide_level loop running...")
            print("DEBUG: Current localtime:", time.localtime())
            if TIDE_DATA and TIME:
                print("DEBUG: TIDE_DATA and TIME are available. Attempting to find closest entry...")
                print(f"DEBUG: TIME (global variable) = {TIME}")
                current_tuple = parse_time_str_with_seconds(TIME)
                try:
                    current_timestamp = time.mktime(current_tuple)
                except:
                    current_timestamp = 0

                closest_entry = None
                smallest_diff = 99999999

                # Find the entry with the minimal time difference
                for entry in TIDE_DATA:
                    entry_tuple = parse_time_str(entry['t'])  # e.g. "2025-01-01 05:36"
                    try:
                        entry_timestamp = time.mktime(entry_tuple)
                    except:
                        entry_timestamp = 0
                    diff = abs(entry_timestamp - current_timestamp)
                    if diff < smallest_diff:
                        smallest_diff = diff
                        closest_entry = entry

                if closest_entry:
                    CURRENT_TIDE_VALUE = closest_entry['v']
                    print(f"Closest tide time: {closest_entry['t']}, tide value: {CURRENT_TIDE_VALUE}")

                    # If tide tracking is on, move the motor
                    if TIDE_TRACKING_MODE and motor:
                        desired_position = int(closest_entry['v'] * motor.max_position)
                        step_count = desired_position - motor.position
                        if step_count != 0:
                            print(f"Moving motor by {step_count} steps to match tide.")
                            await motor.step_async(steps=step_count)
                else:
                    print("DEBUG: No valid closest_entry found.")

            else:
                print("DEBUG: TIDE_DATA or TIME not set. Skipping tide update.")

            print("DEBUG: update_tide_level loop finished. Sleeping 60 seconds.")
            # Sleep 60 seconds before the next check
            await asyncio.sleep(60)

        except Exception as e:
            print(f"Error in update_tide_level: {e}")
            print("DEBUG: Encountered an error, sleeping 60 seconds before retry.")
            await asyncio.sleep(60)

async def check_midnight():
    """Fetch new data at midnight and re-home the motor."""
    while True:
        try:
            # Use TIME variable
            date_time_str = TIME  # Format: YYYY-MM-DD HH:MM:SS
            parts = date_time_str.split(' ')
            if len(parts) == 2:
                time_part = parts[1]
                hour, minute, _ = time_part.split(':')
                if hour == "00" and minute == "00":
                    print("Midnight reached. Rehoming motor and fetching new JSON data...")

                    # --- NEW: Re-home the motor at midnight
                    if motor is not None:
                        await motor.home_async()

                    fetch_json_data()
                    print("New data fetched and normalized.")
                    await asyncio.sleep(60)  # Avoid multiple fetches in the same minute
        except Exception as e:
            print(f"Error in check_midnight: {e}")
        await asyncio.sleep(10)  # Check every 10 seconds

async def position_motor_to_current_tide():
    """
    Immediately finds the closest tide data entry to the current TIME and, if
    TIDE_TRACKING_MODE is enabled, moves the motor to that position.
    """
    global TIDE_TRACKING_MODE, motor, CURRENT_TIDE_VALUE, TIDE_DATA, TIME

    if not TIDE_DATA or not TIME:
        print("No TIDE_DATA or TIME is not set. Skipping immediate positioning.")
        return

    try:
        # Parse current time into a timestamp
        current_tuple = parse_time_str_with_seconds(TIME)
        try:
            current_timestamp = time.mktime(current_tuple)
        except:
            current_timestamp = 0

        # Find the entry with the minimal time difference
        closest_entry = None
        smallest_diff = 99999999
        for entry in TIDE_DATA:
            entry_tuple = parse_time_str(entry['t'])
            try:
                entry_timestamp = time.mktime(entry_tuple)
            except:
                entry_timestamp = 0
            diff = abs(entry_timestamp - current_timestamp)
            if diff < smallest_diff:
                smallest_diff = diff
                closest_entry = entry

        if closest_entry:
            CURRENT_TIDE_VALUE = closest_entry['v']
            print(f"[Immediate Position] Closest tide time: {closest_entry['t']}, tide value: {CURRENT_TIDE_VALUE}")

            # If tide tracking is on, move the motor
            if TIDE_TRACKING_MODE and motor:
                desired_position = int(closest_entry['v'] * motor.max_position)
                step_count = desired_position - motor.position
                if step_count != 0:
                    print(f"[Immediate Position] Moving motor by {step_count} steps to match tide.")
                    await motor.step_async(steps=step_count)
        else:
            print("No valid closest_entry found for immediate positioning.")

    except Exception as e:
        print(f"Error in position_motor_to_current_tide: {e}")

async def handle_client(reader, writer):
    global motor, TIME, TIDE_TRACKING_MODE, CURRENT_TIDE_VALUE, LAST_HOME_TIME
    try:
        request = await reader.read(1024)
        request = request.decode('utf-8')
        print(f"Request: {request}")

        # Parse the HTTP request line
        request_line = request.split('\r\n')[0]
        parts = request_line.split(' ')
        if len(parts) < 2:
            response = "Bad Request"
            content_type = "text/plain"
        else:
            method, path = parts[0], parts[1]
            print(f"Method: {method}, Path: {path}")

            if path.startswith('/home'):
                # Store the current state of tide tracking
                tide_tracking_was_enabled = TIDE_TRACKING_MODE
                if TIDE_TRACKING_MODE:
                    TIDE_TRACKING_MODE = False
                    print("Tide tracking disabled for homing.")

                try:
                    if motor is None:
                        raise ValueError("Motor not initialized.")
                    await motor.home_async()
                    await fetch_time()
                    LAST_HOME_TIME = TIME
                    response = f"{LAST_HOME_TIME}"
                    content_type = "text/plain"
                except Exception as e:
                    print(f"Error handling /home request: {e}")
                    response = "Error homing motor: " + str(e)
                    content_type = "text/plain"
                finally:
                    # Re-enable tide tracking if it was previously enabled
                    if tide_tracking_was_enabled:
                        TIDE_TRACKING_MODE = True
                        print("Tide tracking re-enabled after homing.")

            elif path.startswith('/status'):
                # [Existing /status handling code]
                tide_level = CURRENT_TIDE_VALUE if CURRENT_TIDE_VALUE is not None else "Loading..."
                position = motor.position if motor else "Unknown"
                payload = {
                    "tide": tide_level,
                    "position": position,
                    "lastHomedTime": LAST_HOME_TIME,
                    "tideTracking": TIDE_TRACKING_MODE
                }
                response = ujson.dumps(payload)
                content_type = "application/json"

            elif path.startswith('/maxpos'):
                # [Existing /maxpos handling code]
                try:
                    query = path.split('?')[1]
                    params = query.split('&')
                    new_max = None
                    for param in params:
                        key, value = param.split('=')
                        if key == 'value':
                            new_max = int(value)
                    if new_max is not None and motor:
                        motor.max_position = new_max
                        response = f"max_position set to {new_max}"
                    else:
                        response = "Invalid or missing maxpos value."
                except Exception as e:
                    response = f"Error in setting maxpos: {e}"
                content_type = "text/plain"

            elif path.startswith('/move'):
                # [Existing /move handling code]
                try:
                    query = path.split('?')[1]
                    params = query.split('&')
                    steps = 0
                    for param in params:
                        key, value = param.split('=')
                        if key == 'steps':
                            steps = int(value)
                    # Command the motor to move asynchronously
                    asyncio.create_task(motor.step_async(steps=steps))
                    response = f"Moving {steps} steps."
                except Exception as e:
                    print(f"Error handling /move request: {e}")
                    response = "Invalid move command."
                content_type = "text/plain"

            elif path.startswith('/toggle_tide'):
                # [Existing /toggle_tide handling code]
                TIDE_TRACKING_MODE = not TIDE_TRACKING_MODE

                # If we just switched it ON, immediately adjust position
                if TIDE_TRACKING_MODE:
                    await position_motor_to_current_tide()

                # Build the JSON response
                response = ujson.dumps({"enabled": TIDE_TRACKING_MODE})
                content_type = "application/json"

            else:
                # Serve the main dashboard HTML
                response = HTML
                content_type = "text/html"

        # Prepare and send the HTTP response
        response_headers = (
            "HTTP/1.1 200 OK\r\n"
            f"Content-Type: {content_type}\r\n"
            "Access-Control-Allow-Origin: *\r\n"  # Optional: Allow CORS
            "Connection: close\r\n"
            "\r\n"
        )
        writer.write(response_headers)
        if isinstance(response, str):
            response = response.encode('utf-8')
        writer.write(response)
        await writer.drain()
    except Exception as e:
        print(f"Error handling client: {e}")
    finally:
        await writer.aclose()

async def start_web_server():
    try:
        server = await asyncio.start_server(handle_client, '0.0.0.0', 80)
        print("Web server is running...")
        # Keep the coroutine alive to handle incoming connections
        while True:
            await asyncio.sleep(3600)  # Sleep for an hour and continue
    except Exception as e:
        print(f"Failed to start web server: {e}")

class LimitSwitch:
    def __init__(self, pin):
        try:
            self.pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        except Exception as e:
            print(f"Failed to initialize LimitSwitch on pin {pin}: {e}")

    def state(self):
        try:
            return self.pin.value()
        except Exception as e:
            print(f"Error reading LimitSwitch state: {e}")
            return 1  # Assume not triggered if error

class StepperMotor:
    def __init__(self, pins, limit_switch, sequence=None):
        """
        Initialize the StepperMotor instance.

        Args:
            pins (list): List of GPIO pins controlling the stepper motor.
            sequence (list): Optional step sequence. Defaults to 4-step sequence.
        """
        self.position = 0
        self.max_position = 850
        try:
            self.pins = [Pin(pin, Pin.OUT) for pin in pins]
        except Exception as e:
            print(f"Failed to initialize StepperMotor pins: {e}")
            self.pins = []
        self.limit_switch = limit_switch
        self.sequence = sequence or [
            [1, 0, 0, 1],  # Step 1
            [0, 0, 1, 1],  # Step 2
            [0, 1, 1, 0],  # Step 3
            [1, 1, 0, 0]   # Step 4
        ]
        self.current_step = 0  # Track the current step in the sequence
        self.lock = asyncio.Lock()  # Initialize an asyncio lock

    async def set_pins_async(self, step):
        """
        Set the GPIO pins according to the step sequence.
        """
        try:
            for pin, state in zip(self.pins, step):
                pin.value(state)
        except Exception as e:
            print(f"Error setting motor pins: {e}")

    async def step_async(self, steps=1, delay=0.01, check=True):
        """
        Move the stepper motor a number of steps, bounded by 0 and self.max_position.
        Positive steps move forward, negative steps move backward.
        """
        direction = 1 if steps > 0 else -1
        async with self.lock:
            for _ in range(abs(steps)):
                # Check if next step is within bounds
                next_position = self.position + direction
                if check:
                    if next_position < 0:
                        self.position = 0
                        break
                    elif next_position > self.max_position:
                        self.position = self.max_position
                        break

                self.position = next_position
                self.current_step = (self.current_step + direction) % len(self.sequence)
                await self.set_pins_async(self.sequence[self.current_step])
                await asyncio.sleep(delay)
            
            # --- NEW: Turn off the motor coils after completing movement
            await self.stop_async()

    async def stop_async(self):
        """Turn off all pins to stop the motor."""
        try:
            for pin in self.pins:
                pin.value(0)
        except Exception as e:
            print(f"Error stopping motor: {e}")

    async def home_async(self):
        """Asynchronously move stepper motor to the home position until the limit switch is triggered."""
        print("Homing motor...")
        try:
            
            while not self.limit_switch.state():
                # Move in negative direction until the limit switch is triggered
                await self.step_async(steps=-1, delay=0.05, check=False)
                await asyncio.sleep(0.05)  # Yield control to event loop
            self.position = 0
            print("Motor homed successfully.")
        except Exception as e:
            print(f"Error homing motor: {e}")
            raise

async def periodic_time_sync():
    """Periodically synchronize time with NTP server."""
    while True:
        await fetch_time()
        await asyncio.sleep(60)  # Sync every minute

async def main():
    global motor  # Declare motor as global to access it outside
    wlan = await connect_to_wifi()
    if not wlan:
        print("Cannot proceed without Wi-Fi connection.")
        return

    await fetch_time()
    fetch_json_data()

    # Initialize motor after Wi-Fi and time are set
    limit_switch = LimitSwitch(16)
    motor = StepperMotor([0, 1, 2, 3], limit_switch)
    if motor is None:
        print("Failed to initialize motor. Exiting...")
        return

    # Create background tasks
    asyncio.create_task(check_midnight())
    asyncio.create_task(update_tide_level())
    asyncio.create_task(start_web_server())
    asyncio.create_task(periodic_time_sync())

    # Keep the main function running indefinitely
    while True:
        await asyncio.sleep(3600)  # Sleep for an hour and continue

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping server...")
        try:
            if motor:
                asyncio.run(motor.stop_async())
        except Exception as e:
            print(f"Error stopping motor: {e}")
    except Exception as e:
        print(f"Unhandled exception: {e}")

