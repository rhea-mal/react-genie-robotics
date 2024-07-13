# OptiBots: Motion Capture of Humanoid Robots in Robotic Gaming Simulations Using OptiTrack

## Project Description
This project integrates Optitrack with OpenSAI to simulate a game of volleyball between two robots: the humanoid robot Toro and the Panda robot arm on a mobile base. The user controls Toro using Optitrack, and the Panda arm returns the ball. The game ends when the ball hits the ground. The camera can switch between a global view and Toro's first-person view, allowing for future VR capabilities.

Global View:

https://github.com/rhea-mal/humanoid-motion-mapping/assets/70975260/e486e779-9203-4546-85a5-c2aa4ac2ef25

Head View:

https://github.com/rhea-mal/humanoid-motion-mapping/assets/70975260/262560c0-7174-461a-b65c-12ee78ff18a6

Optitrack Human Interface:


https://github.com/rhea-mal/humanoid-motion-mapping/assets/70975260/1925377f-73c9-4a0b-8065-bd5551e37ef4



### Prerequisites
- OpenSAI: [OpenSAI GitHub Repository](https://github.com/manips-sai-org/OpenSai)
- Motive for Optitrack system

### Optitrack System Setup -  Stanford Robotics Center 
1. **Connect the laptop to the white ethernet cable.**
2. **Set your laptop IP manually to `172.24.68.64`.**

### Motive Configuration
1. **Launch Motive.**
2. **Create rigid bodies:**
   - Select a group of markers.
   - Right-click and create a new rigid body.
   - Rename and re-order the rigid bodies in the Assets tab.

3. **Streaming Settings:**
   - Go to `Edit` -> `Settings`.
   - Click the `Streaming` tab.
   - Change the IP to `172.24.68.48`.
   - Ensure the streaming setting is `Unicast`.
   - Tick `Rigid Bodies` and `Markers` (labeled and unlabeled).
   - Enable streaming.
   - Make sure to disable all streaming before turning off system

### Running the Stream Data Script
1. Navigate to the `cs225a/drivers/PythonClient` directory.
2. Run `StreamData.py` to stream rigid body information to your local Redis server.

### Data Retrieval from Redis Server
- Position keys: `sai2::optitrack::rigid_body_pos::(rigid body number)`
- Orientation keys: `sai2::optitrack::rigid_body_ori::(rigid body number)`
- Note: `(rigid body number)` correlates to the rigid body ordering in the Assets tab.

### Building the Project
Run the following commands:
   ```sh
   mkdir build && cd build
   cmake ..
   make -j4

### Running the Controllers and Simulation
1. Go to the /bin/optitrack/ directory.
2. Run the following commands in separate terminals:
./toro-controller
./panda-controller
./simviz

### Changing Camera View
Use `redis-cli` to change the camera view by setting the camera position Redis key to zero:
```sh
redis-cli