# ROS 2 Tutorials Package

This package contains simple ROS 2 publisher and subscriber nodes to demonstrate the Docker setup.

## Package Contents

- **publisher_node.py** - A node that publishes "Hello from ROS 2!" messages to a topic every second
- **subscriber_node.py** - A node that subscribes to the topic and prints received messages

## Building the Package

Inside the Docker container:

```bash
cd ~/ros2-dev
colcon build --packages-select ros2_tutorials
```

## Running the Nodes

### Terminal 1: Run the Publisher Node

```bash
cd ~/ros2-dev
source install/setup.bash
./install/ros2_tutorials/bin/publisher_node
```

You should see output like:
```
[INFO] [publisher_node]: Publishing: "Hello from ROS 2! Message #1"
[INFO] [publisher_node]: Publishing: "Hello from ROS 2! Message #2"
...
```

### Terminal 2: Run the Subscriber Node

In another terminal inside the Docker container:

```bash
./bash.sh  # If you exited the first terminal
cd ~/ros2-dev
source install/setup.bash
./install/ros2_tutorials/bin/subscriber_node
```

You should see output like:
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

## Next Steps

You can now:
- Create your own ROS 2 packages in `~/ros2-dev/src/`
- Build them with `colcon build`
- Run them with `ros2 run`
- Use other ROS 2 tools like `ros2 topic`, `ros2 node`, etc.

## Stopping the Nodes

Press `Ctrl+C` in any terminal to stop the nodes.
