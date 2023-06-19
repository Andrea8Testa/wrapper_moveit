# wrapper_moveit

### Launch wrapper in simulation

Terminal 1: `
```
roscore
```
Terminal 2 (`hrii_robot_controllers` in branch `ATES/joint_impedance`): 
```
mon launch hrii_fixed_base_robot_controllers joint_impedance_controller.launch robot_id:=panda robot_model:=franka
```
Terminal 3:
```
mon launch wrapper_moveit wrapper_moveit.launch robot_id:=panda
```
Terminal 4: 
```
mon launch panda_moveit_config demo_wrapper.launch robot_id:=panda
```
