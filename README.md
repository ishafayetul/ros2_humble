# ROS 2 Project

Welcome to the ROS 2 Project! This repository contains resources and code for creating and managing ROS 2 workspaces, packages, and implementing various features such as Publisher-Subscriber models, Service Clients, and more.

## Table of Contents
- [Workspace Creation](#workspace-creation)
- [Publisher-Subscriber Model](#publisher-subscriber-model)
- [Timer](#timer)
- [Spinning](#spinning)
- [Custom Messages and Services](#custom-messages-and-services)
- [Service Client](#service-client)
- [Future Object](#future-object)
- [Client Methods](#client-methods)
- [Service Methods](#service-methods)
- [Parameters](#Parameters)

## Workspace Creation

1. **Creating a Workspace**:
   ```bash
   mkdir ~/<ws_name>/src
   ```

2. **Package Creation**:
   ```bash
   ros2 pkg create --build-type ament_python/ament_cpp --license Apache-2.0 <package_name>
   ```

3. **Building the Package**:
   ```bash
   colcon build
   ```
   or to build a specific package:
   ```bash
   colcon build --packages-select <pkg_name>
   ```

4. **Resolving Dependencies**:
   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   ```

5. **Sourcing the Setup File**:
   ```bash
   source install/local_setup.bash
   ```

## Publisher-Subscriber Model

1. **Creating a Node**:
   ```python
   <node_instance_name> = rclpy.create_node(<node_name>)
   ```

2. **Creating a Publisher**:
   ```python
   <publisher_instance_name> = <node_instance>.create_publisher(<msg_type>, <topic_name>, <queue_size>)
   ```

3. **Creating a Subscriber**:
   ```python
   <subscriber_instance_name> = <node_instance>.create_subscription(<msg_type>, <topic_name>, <callBack_function>, <queue_size>)
   ```

4. **Publishing a Message**:
   ```python
   <publisher_instance>.publish(<msg>)
   ```

5. **Printing on Console**:
   ```python
   <node_instance>.get_logger().info("Normal Information")
   <node_instance>.get_logger().warn("Warning")
   <node_instance>.get_logger().info("Error")
   ```

## Timer

Creates a timer that calls a callback function at regular intervals:
```python
<timer_instance> = <node_instance>.create_timer(<intervals>, <callBack_function>)
```

## Spinning

Keeps a node running:
```python
rclpy.spin(<node_instance>)
rclpy.spin_once(<node_instance>)  # Runs only one time
```

## Custom Messages and Services

- File names should start with a capital letter.
- Package type should be `ament_cmake`.
  
To show a custom service:
```bash
ros2 interface show <pkg/srv/file>
```

## Service Client

1. **Creating a Service**:
   ```python
   <service_instance> = <node_instance>.create_service(<srv_msg_name>, <service_name>, <callBack_function>)
   ```

2. **Creating a Client**:
   ```python
   <client_instance> = <node_instance>.create_client(<srv_msg_name>, <service_name>)
   ```

3. **Waiting for Service**:
   ```python
   <client_instance>.wait_for_service(timeout_sec=<value>)
   ```

4. **Creating a Request**:
   ```python
   <request_instance> = <srv_msg>.Request()
   ```

5. **Sending a Request**:
   ```python
   <client_instance>.call_async(<request_instance>)
   ```

6. **Testing a Service from CLI**:
   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
   ```

## Future Object

Represents a value that may not yet be available but will be resolved later.

- **Methods**:
  - `future.done()`: Checks if the future has finished (returns boolean).
  - `future.result()`: Gets the value of response or raises an exception if an error occurs.
  - `future.exception()`: Returns exception if it occurs; otherwise, returns none.
  - `future.cancel()`: Attempts to cancel the request.
  - `future.cancelled()`: Returns true if the future was cancelled.
  - `future.add_done_callback(callback)`: Adds a callback function to be executed once the Future is complete.

## Client Methods

- `client.wait_for_service(timeout_sec=value)`: Waits for the service to become available.
- `client.call(request)`: Sends a service request and blocks until the result is returned.
- `client.call_async(request)`: Sends a service request asynchronously and returns a Future object holding the result.
- `client.destroy()`: Cleans up and destroys the client, releasing resources.
- `client.service_is_ready()`: Checks if the service is available.
- `client.get_service_name()`: Returns the name of the service the client is connected to.
- `client.assert_liveliness()`: Notifies the system that the client is still alive.
- `client.configure_intra_process_communication(enable=True)`: Enables intra-process communication for optimization.

## Service Methods

- `service.destroy()`: Cleans up and destroys the service object, releasing resources.
- `service.service_is_ready()`: Checks if the service is ready to process requests.
- `service.get_service_name()`: Returns the name of the service being handled.
- `service.assert_liveliness()`: Notifies the system that the service is still alive.
- `service.configure_intra_process_communication(enable=True)`: Same as client method.
- `service.get_qos()`: Returns the Quality of Service (QoS) settings for the service.

## Parameters:

- `node.declare_parameter("parameter_name", "optional_default_value")`: Declares a new parameter for the node with a name and optionally a default value.

- `node.set_parameter("param_name", Parameter.Type.INTEGER, <value>)`: Sets the value of a single parameter object.

- `node.set_parameters([ Parameter('param1', Parameter.Type.INTEGER, 10), Parameter('param2', Parameter.Type.STRING, 'value') ])`: Sets multiple parameters' values; also accepts parameter objects.

- `param = node.get_parameter("param_name")`: Returns the parameter object.

- `params = node.get_parameters(['param1', 'param2'])`: Retrieves multiple parameters by name. It returns a list of Parameter objects.

- `param.get_name()`: Returns the name of the parameter.

- `param.get_type()`: Returns the type of the parameter.

- `param.get_parameter_value().string_value`: Returns the parameter value in string type (type casting).

- `param.value`: Returns the actual value (without type casting).

- `param_msg = param.to_parameter_msg()`: Converts the parameter into a message of type Parameter that can be used in communication with other nodes.

- `param = Parameter.from_parameter_msg(param_msg)`: Converts a Parameter message (from communication) back into a Parameter object.

- `repr(param)`: Provides a string representation of the parameter object, useful for debugging.

- `node.has_parameter('param_name')`: Checks if a parameter exists in the node.

- `node.undeclare_parameter('param_name')`: Undeclares a parameter by removing it from the node. After undeclaring, the parameter will no longer be accessible.

- `param = node.get_parameter_or('param_name', Parameter('param_name', Parameter.Type.STRING, 'default_value'))`: Returns the parameter if it exists; otherwise, returns a provided default value.

## Parameter Types:

- `Parameter.Type.BOOL`: Boolean type (True/False).
- `Parameter.Type.INTEGER`: Integer type.
- `Parameter.Type.DOUBLE`: Double/float type.
- `Parameter.Type.STRING`: String type.
- `Parameter.Type.BYTE_ARRAY`: List of bytes.
- `Parameter.Type.INTEGER_ARRAY`: List of integers.
- `Parameter.Type.DOUBLE_ARRAY`: List of doubles.
- `Parameter.Type.STRING_ARRAY`: List of strings.

## Contributing

We welcome contributions! Please feel free to submit issues, fork the repository, and create pull requests.

## License

This project is licensed under the Apache-2.0 License.
