ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "{
header: {
stamp: now
},
name: ['wheel_joint_fl', 'wheel_joint_fr', 'wheel_joint_bl', 'wheel_joint_br', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5'],
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
velocity: [],
effort: []
}"

