Ejecutar
catkin_make
source devel/debug.sh

Parado en cualquier directorio lmwr_ros_model ejecutar:

roslaunch lmwr_ros_model display.launch model:='$(find lmwr_ros_model)/urdf/lmwr.urdf'
(Levanta el modelo en RVIZ con todos los links y Joints)

roslaunch lmwr_ros_model gazebo.launch model:='$(find lmwr_ros_model)/urdf/lmwr-gazebo.urdf.xacro'
(Levanta el modelo en Gazebo)

roslaunch lmwr_ros_model joints.launch model:='$(find lmwr_ros_model)/urdf/lmwr-gazebo.urdf.xacro'
(Levanta el modelo en RVIZ y en Gazebo)

roslaunch lmwr_ros_model diffdrive.launch model:='$(find lmwr_ros_model)/urdf/lmwr-diffdrive.urdf.xacro'
(Levanta el modelo en RVIZ y Gazebo y el control diferencial)

