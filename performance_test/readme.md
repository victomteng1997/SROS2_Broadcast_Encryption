This directory is intended to be an ROS2 workspace. 
Installation and compiling processes follow the default ROS2 documentation.

# Security setup:

```
mkdir ~/sros2_demo
cd ~/sros2_demo
ros2 security create_keystore demo_keys
ros2 security create_key demo_keys /talker_listener/testNode
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

## to run ROS2 with SROS2: 
```
ros2 run py_pubsub ROS2_perf_test --ros-args --enclave /talker_listener/testNode            
```

## to run ROS2 without SROS2, with proposed defense.
```
ros2 run py_pubsub ROS2_perf_test_enc
```

## to run with both:
```
ros2 run py_pubsub ROS2_perf_test_enc --ros-args --enclave /talker_listener/testNode            
```


