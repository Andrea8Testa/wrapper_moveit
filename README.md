# wrapper_moveit

### Launch wrapper in simulation

Terminal 1: `roscore`

Terminal 2: `mon launch hrii_fixed_base_robot_controllers cartesian_impedance_controller.launch robot_id:=panda robot_model:=franka`

Terminal 3: `mon launch wrapper_moveit wrapper_moveit.launch`

Terminal 4: `mon launch panda_moveit_config demo_wrapper.launch arm_id:=panda_franka`
