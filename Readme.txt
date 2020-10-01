Ejecutar
catkin_make
source devel/setup.bash

Parado en cualquier directorio lmwr_ros_model ejecutar:

roslaunch lmwr_ros_model display.launch model:='$(find lmwr_ros_model)/urdf/lmwr.urdf'
(Levanta el modelo en RVIZ con todos los links y Joints)

roslaunch lmwr_ros_model gazebo.launch model:='$(find lmwr_ros_model)/urdf/lmwr-gazebo.urdf.xacro'
(Levanta el modelo en Gazebo)

roslaunch lmwr_ros_model joints.launch model:='$(find lmwr_ros_model)/urdf/lmwr-gazebo.urdf.xacro'
(Levanta el modelo en RVIZ y en Gazebo)

roslaunch lmwr_ros_model diffdrive.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diffdrive.urdf.xacro'
(Levanta el modelo en RVIZ y Gazebo y el control diferencial)

roslaunch lmwr_ros_model diffdrive.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam.urdf.xacro'
(Levanta el modelo con Ultrasonido y camara en RVIZ y Gazebo y el control diferencial)

Para hacer mover el robot hacia adelante a 0.5 m/s por linea de comando:
rostopic pub /lmwr_diff_ive_controller/cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'

Hacia atras:
rostopic pub /lmwr_diff_drive_controller/cmd_vel geometry_msgs/Twist -r 3 -- '[-0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'

Rotación a 0.3 rad/s
rostopic pub /lmwr_diff_drive_controller/cmd_vel geometry_msgs/Twist -r 3 -- '[0.0,0.0,0.0]' '[0.0, 0.0, 0.3]'



roslaunch lmwr_ros_model walker.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam.urdf.xacro'
(Levanta el modelo con Ultrasonido y camara en RVIZ y Gazebo y el nodo walker)

roslaunch lmwr_ros_model walker.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam.urdf.xacro' world:='$(find lmwr_ros_model)/worlds/lmwr.world'
(Levanta el modelo con Ultrasonido y camara en RVIZ y Gazebo y el nodo walker)

roslaunch lmwr_ros_model walker.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam.urdf.xacro' world:='worlds/willowgarage.world'
(Levanta el modelo con Ultrasonido y camara en RVIZ y Gazebo y el nodo walker. En Gazebo levanta el Willowgarage World)

roslaunch lmwr_ros_model walker.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam.urdf.xacro' world:='$(find lmwr_ros_model)/worlds/lmwr.world'

roslaunch lmwr_ros_model walker.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam-v3.urdf.xacro' world:='$(find lmwr_ros_model)/worlds/garden_2_lmwr.world'

roslaunch lmwr_ros_model walker.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diff-cam-v3.urdf.xacro' world:='$(find lmwr_ros_model)/worlds/garden_3_lmwr.world'

