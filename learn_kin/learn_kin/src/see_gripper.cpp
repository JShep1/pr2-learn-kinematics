#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>

#include <tf/tf.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <string.h>


void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z << " with orientation "
      << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y
      << feedback->pose.orientation.z << ", " << feedback->pose.orientation.w);
}

visualization_msgs::InteractiveMarker makeGripperMarker(const char *name, const geometry_msgs::PoseStamped &stamped)
{
  //create an interactive marker from with the pose and name we want 
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.name = name;
  int_marker.description = "*";
  int_marker.scale = 1.0; //scale;
  int_marker.pose = stamped.pose;

  //we create the control for this gripper marker
  /*we need to do it this way because the control defines an array of markers
    and since the hand/gripper is composed of 3 markers/components this is the easiest way
    to define it, especially since the their pose will be interpreted as relative to the 
    pose of the parent interactive marker.
  */
  visualization_msgs::InteractiveMarkerControl control;
  //NONE: This control is only meant for visualization; no context menu, and no actions can be performed on it.
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  //the contained markers will also be visible
  control.always_visible = true;

  //we define a marker mesh to add to the marker control
  visualization_msgs::Marker mesh;
  //basic setings that defines the marker caracteristics from a mesh
  mesh.mesh_use_embedded_materials = true;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  //we want to preserve the scale of the mesh
  mesh.scale.x = 1.0;
  mesh.scale.y = 1.0;
  mesh.scale.z = 1.0;
  
  //no need to define color since we are using the mesh color -- maybe change this later?
  //mesh.color = color; //std_msgs::ColorRGBA color
  
  //first marker component the gripper palm
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
  mesh.pose.orientation.w = 1;
  control.markers.push_back( mesh );

  //this represents how much the gripper will be open
  float gripper_angle = 0.3;
  
  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;
  // set the pose of the first finger:
  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0,0,1),  gripper_angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -gripper_angle));
  T_proximal = T1;
  T_distal = T1 * T2;
  
  //second & third markers make a finger
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  tf::poseTFToMsg (T_proximal, mesh.pose);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  tf::poseTFToMsg(T_distal, mesh.pose);
  control.markers.push_back( mesh );

  //set the orientations for the second finger
  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI)*tf::Quaternion(tf::Vector3(0,0,1),  gripper_angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -gripper_angle));
  T_proximal = T1;
  T_distal = T1 * T2;
  
  //forth & fifth markers make the finger
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  tf::poseTFToMsg(T_proximal, mesh.pose);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  tf::poseTFToMsg(T_distal, mesh.pose);
  control.markers.push_back( mesh );

  //add the control for the interaction marker
  int_marker.controls.push_back( control );
  return int_marker;

}  

visualization_msgs::InteractiveMarker makeGripperMarkerI(const char *name, const geometry_msgs::PoseStamped &stamped)
{
  //create an interactive marker from with the pose and name we want 
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.name = name;
  int_marker.description = "+";
  int_marker.scale = 1.0; //scale;
  int_marker.pose = stamped.pose;

  //we create the control for this gripper marker
  /*we need to do it this way because the control defines an array of markers
    and since the hand/gripper is composed of 3 markers/components this is the easiest way
    to define it, especially since the their pose will be interpreted as relative to the 
    pose of the parent interactive marker.
  */
  visualization_msgs::InteractiveMarkerControl control;
  //NONE: This control is only meant for visualization; no context menu, and no actions can be performed on it.
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  //the contained markers will also be visible
  control.always_visible = true;

  //we define a marker mesh to add to the marker control
  visualization_msgs::Marker mesh;
  //basic setings that defines the marker caracteristics from a mesh
  mesh.mesh_use_embedded_materials = true;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  //we want to preserve the scale of the mesh
  mesh.scale.x = 1.0;
  mesh.scale.y = 1.0;
  mesh.scale.z = 1.0;
  
  //no need to define color since we are using the mesh color -- maybe change this later?
  //mesh.color = color; //std_msgs::ColorRGBA color
  
  //first marker component the gripper palm
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
  mesh.pose.orientation.w = 1;
  control.markers.push_back( mesh );

  //this represents how much the gripper will be open
  float gripper_angle = 0.3;
  
  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;
  // set the pose of the first finger:
  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0,0,1),  gripper_angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -gripper_angle));
  T_proximal = T1;
  T_distal = T1 * T2;
  
  //second & third markers make a finger
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  tf::poseTFToMsg (T_proximal, mesh.pose);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  tf::poseTFToMsg(T_distal, mesh.pose);
  control.markers.push_back( mesh );

  //set the orientations for the second finger
  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI)*tf::Quaternion(tf::Vector3(0,0,1),  gripper_angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -gripper_angle));
  T_proximal = T1;
  T_distal = T1 * T2;
  
  //forth & fifth markers make the finger
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  tf::poseTFToMsg(T_proximal, mesh.pose);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  tf::poseTFToMsg(T_distal, mesh.pose);
  control.markers.push_back( mesh );

  //add the control for the interaction marker
  int_marker.controls.push_back( control );
  return int_marker;

}  


visualization_msgs::Marker makeQRMarker(const char *name, const geometry_msgs::PoseStamped &stamped)
{
  visualization_msgs::Marker marker;
  marker.header = stamped.header;
  marker.ns = name;
  marker.id = 0;
  // Set the shape we what the marker to be
  marker.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = stamped.pose;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  //never to auto-delete
  marker.lifetime = ros::Duration();
  
  //set the marker color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0; // this has to me non-zero, 0 means completly transparent/invisible 
  
  return marker;
}

visualization_msgs::InteractiveMarker makeQRIntMarker(const char *name, const geometry_msgs::PoseStamped &stamped)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.name = "temp";
  size_t name_len = strlen(name)+1;
  std::string new_name = new char[name_len];
  for(int i = 0; i < name_len;i++){
    new_name[i] = name[i];
  }
  int_marker.description = "t";
  int_marker.pose = stamped.pose;
  
  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.always_visible = true;
  marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  //never to auto-delete
  marker.lifetime = ros::Duration();
  
  //set the marker color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0; // this has to be non-zero, 0 means completly transparent/invisible 
  
  marker_control.markers.push_back(marker);
  
  int_marker.controls.push_back(marker_control);
  
  
  return int_marker;
}

visualization_msgs::InteractiveMarker makeQRIntMarkerCircle(const char *name, const geometry_msgs::PoseStamped &stamped)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.name = "temp";
  size_t name_len = strlen(name)+1;
  std::string new_name = new char[name_len];
  for(int i = 0; i < name_len;i++){
    new_name[i] = name[i];
  }
  int_marker.description = "rot_origin";
  int_marker.pose = stamped.pose;
  
  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.always_visible = true;
  marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  //never to auto-delete
  marker.lifetime = ros::Duration();
  
  //set the marker color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0; // this has to be non-zero, 0 means completly transparent/invisible 
  
  marker_control.markers.push_back(marker);
  
  int_marker.controls.push_back(marker_control);
  
  
  return int_marker;
}

void createCircularArcPoses(const geometry_msgs::PoseStamped &start_point, std::vector<geometry_msgs::Pose>& out_poses, double angle, double radius, int noOfPoses, Eigen::Vector3d rotation, bool keep_orientation = true)
{
  //determine the rotation axis


  //double direction = angle > 0.0 ? 1.0: -1.0;
  double angular_resolution = angle/noOfPoses;
  // I am adding the rotation center to the poses so I can show it in rviz
  out_poses.resize(noOfPoses+1);
  
  Eigen::Affine3d start;
  tf::poseMsgToEigen(start_point.pose, start);
  
  //Eigen::Translation3d rotation_center (radius*Eigen::Vector3d::UnitY());
  //compute the rotation center
  geometry_msgs::Pose rotation_pose;
  rotation_pose.orientation.x = start_point.pose.orientation.x; 
  rotation_pose.orientation.y = start_point.pose.orientation.y; 
  rotation_pose.orientation.z = start_point.pose.orientation.z; 
  rotation_pose.orientation.w = start_point.pose.orientation.w; 
  rotation_pose.position.x = start_point.pose.position.x + radius;// + 0.185;
  rotation_pose.position.y = start_point.pose.position.y;// + radius;
  rotation_pose.position.z = start_point.pose.position.z;
  Eigen::Affine3d rotation_center;
  tf::poseMsgToEigen(rotation_pose, rotation_center);
  out_poses[0] = rotation_pose;
  Eigen::Translation3d translate(start_point.pose.position.x * Eigen::Vector3d::UnitX());
  
  for (int i = 1; i <= noOfPoses; ++i)
  {
    Eigen::Affine3d rotation_increment (Eigen::AngleAxisd(angular_resolution*i, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d pose (rotation_center*rotation_increment*rotation_center.inverse()*start); 

    tf::poseEigenToMsg(pose,out_poses[i]);  

    if (keep_orientation)
    {
      //out_poses.at(i).orientation =  start_point.pose.orientation;
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gripper_shape_marker");
  ros::NodeHandle n;

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("marker");
  
  //create the pose we want the gripper to be in
  geometry_msgs::PoseStamped gripper_pose; 
  gripper_pose.header.frame_id ="/odom_combined";
  gripper_pose.header.stamp = ros::Time::now();
  
  /*gripper_pose.pose.position.x = 0.702365;//0.919967996904-0.185;
  gripper_pose.pose.position.y = 0.248003;
  gripper_pose.pose.position.z =  0.667633;
  
  gripper_pose.pose.orientation.x = -0.0296479;
  gripper_pose.pose.orientation.y = 0.0498673;
  gripper_pose.pose.orientation.z = -0.483921;
  gripper_pose.pose.orientation.w = 0.873187;*/
  
  gripper_pose.pose.position.x = 0.562671;
  gripper_pose.pose.position.y = -0.437958;
  gripper_pose.pose.position.z = 0.776837;
  
  gripper_pose.pose.orientation.x = 0.0166176;
  gripper_pose.pose.orientation.y = -0.0149043;
  gripper_pose.pose.orientation.z = 0.257844;
  gripper_pose.pose.orientation.w = 0.965929;
  
  //create the visualization gripper marker
  visualization_msgs::InteractiveMarker gripper_marker = makeGripperMarker("start_point", gripper_pose);

  std::vector<geometry_msgs::Pose> out_poses;
  createCircularArcPoses(gripper_pose,out_poses,-1.57, 0.5, 4, Eigen::Vector3d::UnitZ());
  
   // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(gripper_marker);//, &processFeedback);
  
    geometry_msgs::PoseStamped center_pose; 
    center_pose.header.frame_id ="/odom_combined";
    center_pose.header.stamp = ros::Time::now();
    center_pose.pose = out_poses.at(0);
    visualization_msgs::InteractiveMarker center_marker = makeQRIntMarkerCircle("center", center_pose);
    server.insert(center_marker);

  for (int i = 1; i<5; i++){
    geometry_msgs::PoseStamped interm_pose; 
    interm_pose.header.frame_id ="/odom_combined";
    interm_pose.header.stamp = ros::Time::now();
    interm_pose.pose = out_poses.at(i);
    visualization_msgs::InteractiveMarker gripper = makeGripperMarkerI("at "+i, interm_pose);
    server.insert(gripper);
    //server.applyChanges();
  }
  
  server.applyChanges();

  // we keep orientation
  /*
  gripper_pose.pose.position.x = 0.0;//0.919967996904-0.185;
  gripper_pose.pose.position.y = 0.0;
  gripper_pose.pose.position.z =  0.667633;
  gripper_pose.pose.orientation.x = -0.0296479;
  gripper_pose.pose.orientation.y = 0.0498673;
  gripper_pose.pose.orientation.z = -0.483921;
  gripper_pose.pose.orientation.w = 0.873187;

  visualization_msgs::InteractiveMarker gripper_marker2 = makeGripperMarker("start_point2", gripper_pose);
  server.insert(gripper_marker2);
  center_pose.header.frame_id ="";
  center_pose.header.stamp = ros::Time::now();
  center_pose.pose = out_poses.at(0);
  visualization_msgs::InteractiveMarker center_marker2 = makeQRIntMarkerCircle("center2", center_pose);
  server.insert(center_marker2);

 createCircularArcPoses(gripper_pose,out_poses,1.57, -0.5, 4,Eigen::Vector3d::UnitZ());
  for (int i = 1; i<5; i++){
    geometry_msgs::PoseStamped interm_pose; 
    interm_pose.header.frame_id ="";
    interm_pose.header.stamp = ros::Time::now();
    interm_pose.pose = out_poses.at(i);
    visualization_msgs::InteractiveMarker gripper2 = makeGripperMarkerI("at2 "+i, interm_pose);
    server.insert(gripper2);
    //server.applyChanges();
  }
  

  server.applyChanges();*/

  //create the pose we have for the QR marker
  /*geometry_msgs::PoseStamped qr_pose; 
  qr_pose.header.frame_id ="/odom_combined";
  qr_pose.header.stamp = ros::Time::now();
 
  qr_pose.pose.position.x = 0.0;
  qr_pose.pose.position.y = 0.0;
  qr_pose.pose.position.z = 0.232;
  
  qr_pose.pose.orientation.x = 0;
  qr_pose.pose.orientation.y = 0;
  qr_pose.pose.orientation.z = 0;
  qr_pose.pose.orientation.w = 1;
  
  //ros::Publisher QR_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::InteractiveMarker QR_tag = makeQRIntMarker("QR_tag", qr_pose);
  server.insert(QR_tag);
  server.applyChanges();
  //QR_pub.publish(QR_tag);*/
  
  // start the ROS main loop
  ros::spin();
}
 
  
  
  
  
