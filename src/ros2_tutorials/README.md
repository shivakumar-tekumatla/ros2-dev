# ROS 2 Tutorials Package

This package contains simple ROS 2 publisher and subscriber nodes with launch file support.

## Package Contents

### Nodes
- **publisher_node.py** - Publishes "Hello from ROS 2!" messages to a topic every second
- **subscriber_node.py** - Subscribes to the topic and prints received messages

### Launch Files
- **tutorial_launch.py** - Launches both publisher and subscriber nodes together
- **publisher_launch.py** - Launches only the publisher node
- **subscriber_launch.py** - Launches only the subscriber node

## Building the Package

Inside the Docker container:

```bash
cd ~/ros2-dev
colcon build --packages-select ros2_tutorials
source install/setup.bash
```

## Running the Nodes

### Option 1: Using ros2 run (Individual nodes)

```bash
# Terminal 1 - Publisher
ros2 run ros2_tutorials publisher_node

# Terminal 2 - Subscriber (in another terminal)
ros2 run ros2_tutorials subscriber_node
```

### Option 2: Using ros2 launch (Recommended)

**Launch both nodes together:**
```bash
ros2 launch ros2_tutorials tutorial_launch.py
```

**Launch nodes separately:**
```bash
# Terminal 1 - Publisher only
ros2 launch ros2_tutorials publisher_launch.py

# Terminal 2 - Subscriber only
ros2 launch ros2_tutorials subscriber_launch.py
```

## Expected Output

**Publisher:**
```
[INFO] [publisher_node]: Publishing: "Hello from ROS 2! Message #1"
[INFO] [publisher_node]: Publishing: "Hello from ROS 2! Message #2"
...
```

**Subscriber:**
```
[INFO] [subscriber_node]: I heard: "Hello from ROS 2! Message #1"
[INFO] [subscriber_node]: I heard: "Hello from ROS 2! Message #2"
...
```

## Verifying the Setup

If everything is working correctly, you should see:
- Publisher printing messages every second
- Subscriber receiving and printing those messages

This confirms that:
1. ✅ Docker environment is set up correctly
2. ✅ ROS 2 Humble is installed and working
3. ✅ Python support is available
4. ✅ rclpy (ROS 2 Python client library) is functional
5. ✅ Topic communication works
6. ✅ Launch files are properly configured

## Useful ROS 2 Commands

```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic

# Get info about a topic
ros2 topic info /topic

# List available executables in this package
ros2 pkg executables ros2_tutorials

# List available launch files
ros2 pkg list
```

## Package Structure

```
ros2_tutorials/
├── package.xml           # Package metadata and dependencies
├── setup.py             # Python package setup
├── setup.cfg            # Setup configuration
├── resource/            # Package marker for ament
│   └── ros2_tutorials
├── ros2_tutorials/      # Python package source
│   ├── __init__.py
│   ├── publisher_node.py
│   └── subscriber_node.py
└── launch/              # Launch files
    ├── tutorial_launch.py
    ├── publisher_launch.py
    └── subscriber_launch.py
```

## Next Steps

You can now:
- Create your own ROS 2 packages in `~/ros2-dev/src/`
- Build them with `colcon build`
- Run them with `ros2 run` or `ros2 launch`
- Use other ROS 2 tools like `ros2 topic`, `ros2 node`, etc.

## Troubleshooting

**"No executable found" error:**
- Make sure you've sourced the workspace: `source install/setup.bash`
- Rebuild the package: `colcon build --packages-select ros2_tutorials`

**Nodes don't communicate:**
- Check if both nodes are running: `ros2 node list`
- Verify the topic exists: `ros2 topic list`
- Check topic type: `ros2 topic info /topic`

## Stopping the Nodes

Press `Ctrl+C` in the terminal to stop the nodes.
