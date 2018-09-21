#include <learn_kin/learn_kin.h>
#include <string>
#include <sstream>
#include <rosbag/bag.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


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






Gripper::Gripper()
{

	//Initialize the client for the Action interface to the gripper controller
	//and tell the action client that we want to spin a thread by default
	gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
	contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
	force_client_  = new ForceClient("r_gripper_sensor_controller/force_servo",true);
	event_detector_client_ = new EventDetectorClient("r_gripper_sensor_controller/event_detector",true);    
	slip_client_ = new SlipClient("r_gripper_sensor_controller/slip_servo", true);

	//wait for the gripper action server to come up 
	while(!gripper_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_sensor_controller/gripper_action action server to come up");
	}

	while(!contact_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_sensor_controller/find_contact action server to come up");
	}

	while(!force_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_sensor_controller/force_servo action server to come up");
	}

	while(!event_detector_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_sensor_controller/event_detector action server to come up");
	}

	while(!slip_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_sensor_controller/slip_servo action server to come up");
	}

}

Gripper::~Gripper()
{
	delete gripper_client_;
	delete contact_client_;
	delete force_client_;
	delete slip_client_;
	delete event_detector_client_;
}

void Gripper::open(){
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
	open.command.position = 0.09;
	open.command.max_effort = -1.0;

	ROS_INFO("Sending open goal");
	gripper_client_->sendGoal(open);
	gripper_client_->waitForResult();
	if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("The gripper opened!");
	else
		ROS_INFO("The gripper failed to open.");
}


//Hold somethign with a constant force in the gripper
void Gripper::hold(double holdForce){
	pr2_gripper_sensor_msgs::PR2GripperForceServoGoal squeeze;
	squeeze.command.fingertip_force = holdForce;   // hold with X N of force

	ROS_INFO("Sending hold goal");
	force_client_->sendGoal(squeeze);
	force_client_->waitForResult();
	if(force_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Stable force was achieved");
	else
		ROS_INFO("Stable force was NOT achieved");
}

//Find two contacts and go into force control mode
void Gripper::findTwoContacts(){
	pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo;
	findTwo.command.contact_conditions = findTwo.command.BOTH;  // close until both fingers contact
	findTwo.command.zero_fingertip_sensors = true;   // zero fingertip sensor values before moving


	ROS_INFO("Sending find 2 contact goal");
	contact_client_->sendGoal(findTwo);
	contact_client_->waitForResult(ros::Duration(5.0));
	if(contact_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Contact found. Left: %d, Right: %d", contact_client_->getResult()->data.left_fingertip_pad_contact, 
				contact_client_->getResult()->data.right_fingertip_pad_contact);
		ROS_INFO("Contact force. Left: %f, Right: %f", contact_client_->getResult()->data.left_fingertip_pad_force, 
				contact_client_->getResult()->data.right_fingertip_pad_force);
		//TODO investigate contact client properties of getresult
	}
	else
		ROS_INFO("The gripper did not find a contact or could not maintain contact force.");
}

void Gripper::slipServo(){
	pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal slip_goal;

	ROS_INFO("Slip servoing");
	slip_client_->sendGoal(slip_goal);
	if(slip_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You should never see this message");
	}else{
		ROS_INFO("SlipServo Action returned without success");
	}
}

void Gripper::place(){

	pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place_goal;
	//TODO investigate below options for trigger conditions
	place_goal.command.trigger_conditions = place_goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;  
	place_goal.command.acceleration_trigger_magnitude = 4.0; // set the contact acceleration to n m/s^2
	place_goal.command.slip_trigger_magnitude = 0.005;

	ROS_INFO("Waiting for object placement contact...");

	event_detector_client_->sendGoal(place_goal);
	event_detector_client_->waitForResult();
	if(event_detector_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Place Success");
		ROS_INFO("cond met: %d, acc_met: %d, slip_met: %d", event_detector_client_->getResult()->data.trigger_conditions_met, event_detector_client_->getResult()->data.acceleration_event, event_detector_client_->getResult()->data.slip_event);
	}else{
		ROS_INFO("Place Failure");
	}

}

LearnKinematics::LearnKinematics()
{
	l_hand_enabled = false;
	r_hand_enabled = true;

	robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");
	/* Get a shared pointer to the model */
	robot_model_ = robot_model_loader_.getModel();

	// Create a kinematic state - this represents the configuration for the robot 
	kinematic_state_.reset(new robot_state::RobotState(robot_model_));
	//init the kinematic state first
	kinematic_state_->setToDefaultValues();

	/* Get the configuration for the joints for the arm of the PR2*/
	if(l_hand_enabled)
	{
		joint_state_group_ = kinematic_state_->getJointStateGroup("left_arm");
	}
	else
	{
		joint_state_group_ = kinematic_state_->getJointStateGroup("right_arm");
	}
}

LearnKinematics::~LearnKinematics()
{
	/*
	   delete traj_client_;
	   delete contact_client_;
	   delete gripper_client_;
	   delete force_client_;
	 */
}

/*
 * helper print functions
 */

void LearnKinematics::print_transform(tf::Transform trans, std::string info)
{
	ROS_INFO_STREAM(info << " position: " << trans.getOrigin().getX() << " "
			<< trans.getOrigin().getY() << " "<< trans.getOrigin().getZ());
	ROS_INFO_STREAM(info <<" orientation: " << trans.getRotation().getX() << " "
			<< trans.getRotation().getY() << " "<< trans.getRotation().getZ() << " "
			<< trans.getRotation().getW());
}

void LearnKinematics::print_pose(geometry_msgs::Pose pose, std::string info)
{
	ROS_INFO_STREAM(info << " position: " << pose.position.x << " "
			<< pose.position.y << " "<< pose.position.z);
	ROS_INFO_STREAM(info << " orientation: " << pose.orientation.x << " "
			<< pose.orientation.y << " "<< pose.orientation.z << " "
			<< pose.orientation.w);
}

//To be used for opening and closing, takes a position
//and max_effort and sends the gripper to that pos with that
//effort and returns if successful or not
bool LearnKinematics::open(GripperClient* gripper_client_, double position, double max_effort){
	bool success = false;
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
	open.command.position = position;
	open.command.max_effort = max_effort;


	ROS_INFO("Sending open goal");

	gripper_client_->sendGoal(open);
	gripper_client_->waitForResult();

	if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Gripper success");
		success = true;
	}else{
		ROS_INFO("Gripper failure");
	}

	gripper_client_->cancelAllGoals();

	return success;
}

//function to get the width of the object within pepe's grasp,
//accurate to the value of epsilon
double LearnKinematics::get_width(GripperClient* gripper_client_, double eps){
	double width = 0.08;
	while (width >= 0){
		if(!open(gripper_client_,width,-1.0)){//do not limit effort of grasp
			//if(!open(gripper_client_,width,50.0)){//Close gently
			ROS_INFO_STREAM("Object of length " << (100*(width+eps)) << " cm detected.");
			return width+eps;
		}
		width -= eps;
		sleep(1.0);//sleep is needed
		}

		ROS_INFO("No object detected. Reopening gripper.");
		open(gripper_client_, 0.08, -1.0);
		return 0.0;
	}

	void LearnKinematics::initialize()
	{
		ROS_INFO("instantiating LearnKinematics");
		//by default we are running this test with the right hand

		//init the clients for the gripper and arm joints
		/*
		   if(l_hand_enabled)
		   {
		   contact_client_  = new ContactClient("l_gripper_sensor_controller/find_contact",true);
		   gripper_client_ = new GripperClient("l_gripper_sensor_controller/gripper_action", true);
		   force_client_  = new ForceClient("l_gripper_sensor_controller/force_servo",true);
		   traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
		   }
		   else
		   {
		   contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
		   gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
		   force_client_  = new ForceClient("r_gripper_sensor_controller/force_servo",true);
		   traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
		   }
		 */

		// initialize the listeners
		/*
		   joint_states_sub_ = nh.subscribe<sensor_msgs::JointState>("joint_states",1,boost::bind(&GoToPose::jointStatesCallback, this, _1));
		   robot_state_.reset(new sensor_msgs::JointState());
		   got_robot_state = false;


		//wait for the different clients action servers to come up
		while(!contact_client_->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the r/l _gripper_sensor_controller/find_contact action server to come up");
		while(!gripper_client_->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the r/l _gripper_sensor_controller/gripper_action action server to come up");
		while(!force_client_->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the r/l _gripper_sensor_controller/force_servo action server to come up");        
		while(!traj_client_->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the joint_trajectory_action server");
		 */

		robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");
		/* Get a shared pointer to the model */
		robot_model_ = robot_model_loader_.getModel();

		// Create a kinematic state - this represents the configuration for the robot 
		kinematic_state_.reset(new robot_state::RobotState(robot_model_));
		//init the kinematic state first
		kinematic_state_->setToDefaultValues();

		/* Get the configuration for the joints for the arm of the PR2*/
		if(l_hand_enabled)
		{
			joint_state_group_ = kinematic_state_->getJointStateGroup("left_arm");
		}
		else
		{
			joint_state_group_ = kinematic_state_->getJointStateGroup("right_arm");
		}

		/* maybe we need this

		// define the source and target frames for the tf transforms
		targetFrame_ = "odom_combined";

		// following is a simple test to see the effect of T from IR  
		sourceFrame_ = "head_mount_kinect_ir_optical_frame";

		transform_ready = false;
		 */
	}

	/*
	   trajectory_msgs::JointTrajectoryPointPtr LearnKinematics::create_traj_point(std::vector<double> joint_values, double time)
	   {
	   trajectory_msgs::JointTrajectoryPointPtr traj_point(new trajectory_msgs::JointTrajectoryPoint);
	//traj_point->positions.resize(joint_values.size());
	traj_point->time_from_start = ros::Duration(time);

	for (unsigned int i = 0; i < joint_values.size(); ++i)
	{
	//ROS_INFO("IK sol %s: %f",joint_names[i].c_str(), joint_values[i]);
	//set the goal positions to be the ones the IK found
	traj_point->positions.push_back(joint_values[i]);
	traj_point->accelerations.push_back(0.0);
	traj_point->velocities.push_back(0.0);
	}
	return  traj_point;
	}



	void LearnKinematics::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
	{
	//ROS_INFO("LearnKinematics::jointStatesCallback");
	robot_state_ = msg;
	if(robot_state_)
	got_robot_state = true;
	}

	//given a end effector pose for the right/left arm this attempts to provide an IK solutions
	// keeping it as a backup for try_action
	bool LearnKinematics::getIK(geometry_msgs::Pose eef_pose)
	{
	//print_pose(eef_pose, "End effector for IK");

	std::cout<< "[LearnKinematics::getIK] got_robot_state:  " << got_robot_state << std::endl;

	if(got_robot_state){
// Updated the robot joints to the latest state
kinematic_state_->setStateValues(*robot_state_);

}
else
ROS_WARN("LearnKinematics::getIK: We are using the default robot joint state NOT current ");

Eigen::Affine3d pose_in;
tf::poseMsgToEigen(eef_pose,pose_in);

std::cout<< "[LearnKinematics::getIK] before joint_state_group_->setFromIK "<< std::endl;

bool found_ik = joint_state_group_->setFromIK (pose_in,10,0.5);
ROS_INFO("LearnKinematics::getIK: We found an IK solution %d",found_ik);
return found_ik;
}*/

//use this to get the pose of the end effector in the odom_combined frame
geometry_msgs::PoseStamped LearnKinematics::get_eef_pose(){
	geometry_msgs::PoseStamped eef_current_pose; 

	if(l_hand_enabled)
	{
		moveit::planning_interface::MoveGroup group("left_arm");
		eef_current_pose = group.getCurrentPose();
		//ROS_INFO_STREAM("Left Arm selected" );
		//ROS_INFO_STREAM("Desired pose:" << eef_current_pose);

	}
	else
	{
		moveit::planning_interface::MoveGroup group("right_arm");
		eef_current_pose = group.getCurrentPose();
		//ROS_INFO_STREAM("Right Arm selected" );
		//ROS_INFO_STREAM("Desired pose:" << eef_current_pose);
	}
	//TODO: return or update global var eef_current_pose
	return eef_current_pose; 
} 

/*
 * TODO: replace this to accomodate different types of acctions: pull/push, rotate 
 * add a type variable
 */
geometry_msgs::Pose LearnKinematics::compute_test_pose(double displacement)
{
	geometry_msgs::Pose pose = get_eef_pose().pose;
	//this is a pull acction
	//pose.position.y = pose.position.y + displacement; 
	//pose.position.z = pose.position.z + displacement; 
	pose.position.x = pose.position.x + displacement; 
	ROS_INFO("Pull pose << pose");
	return pose;
}

void LearnKinematics::try_action(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroup* group)
{
	//const std::vector<std::string> &joint_names = joint_state_group_->getJointModelGroup()->getJointModelNames();

	//moveit::planning_interface::MoveGroup* group;


	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("EEF link frame: %s", group->getEndEffectorLink().c_str());

	moveit::planning_interface::MoveGroup::Plan my_plan;

	group->setPlanningTime(5.0);//was 30


	//TODO: update below line with a pose taken in as an argument
	// geometry_msgs::Pose target_pose = compute_test_pose(displacement);

	group->setPoseTarget(target_pose);

	bool success = group->plan(my_plan);
	ROS_INFO("Visualizing plan  (pose goal): %s",success?"OK I can plan it":"FAILED planning"); 
	sleep(0.5);//was 5

	if(success)   
	{
		//this waits for the execution of the trajectory to complete.
		bool moved = group->asyncExecute(my_plan);
		ROS_INFO("moved  (pose goal): %s",moved?"OK I moved":"FAILED moving");
		//sleep(2);
		//TODO enable the below line if you want the action to stop execution
		//group->stop();
		//ROS_INFO("moved  (pose goal): %s",moved?"OK I moved after 2":"FAILED moving");
		//ROS_INFO_STREAM("End pose:" << group->getCurrentPose());

	}
	else
	{
		ROS_INFO(" DID NOT call group->move() "); 
	}
	//sleep(3); // wait after the move 3 seconds
}

//
void LearnKinematics::try_action(double displacement, moveit::planning_interface::MoveGroup* group)
{
	//const std::vector<std::string> &joint_names = joint_state_group_->getJointModelGroup()->getJointModelNames();

	//moveit::planning_interface::MoveGroup* group;


	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("EEF link frame: %s", group->getEndEffectorLink().c_str());

	moveit::planning_interface::MoveGroup::Plan my_plan;

	group->setPlanningTime(5.0);//was 30


	//TODO: update below line with a pose taken in as an argument
	geometry_msgs::Pose target_pose = compute_test_pose(displacement);

	group->setPoseTarget(target_pose);

	bool success = group->plan(my_plan);
	ROS_INFO("Visualizing plan  (pose goal): %s",success?"OK I can plan it":"FAILED planning"); 
	sleep(0.5);//was 5

	if(success)   
	{
		//this waits for the execution of the trajectory to complete.
		bool moved = group->asyncExecute(my_plan);
		ROS_INFO("moved  (pose goal): %s",moved?"OK I moved":"FAILED moving");
		//sleep(2);
		//TODO enable the below line if you want the action to stop execution
		//group->stop();
		//ROS_INFO("moved  (pose goal): %s",moved?"OK I moved after 2":"FAILED moving");
		//ROS_INFO_STREAM("End pose:" << group->getCurrentPose());

	}
	else
	{
		ROS_INFO(" DID NOT call group->move() "); 
	}
	//sleep(3); // wait after the move 3 seconds
}

//finds distance between two 3d vectors
double find_diff(geometry_msgs::PoseStamped first_pose, geometry_msgs::PoseStamped second_pose){

	double first_x = first_pose.pose.position.x;
	double first_y = first_pose.pose.position.y;
	double first_z = first_pose.pose.position.z;
	double second_x = second_pose.pose.position.x;
	double second_y = second_pose.pose.position.y;
	double second_z = second_pose.pose.position.z;

	double distance = pow((pow((first_x-second_x),2)+pow((first_y-second_y),2)+pow((first_z-second_z),2)),0.5);

	return distance;
}

double find_diff(double first_pose[], double second_pose[]){
	double first_x = first_pose[0];
	double first_y = first_pose[1];
	double first_z = first_pose[2];
	double second_x = second_pose[0];
	double second_y = second_pose[1];
	double second_z = second_pose[2];

	double distance = pow((pow((first_x-second_x),2)+pow((first_y-second_y),2)+pow((first_z-second_z),2)),0.5);

	return distance;

}

void copy_over(double first_pose[], double second_pose[], int length){
	for(int i = 0; i < length; ++i){
		first_pose[i] = second_pose[i];
	}
}

void populate_coords(double array_pose[], geometry_msgs::PoseStamped pose){
	array_pose[0] = pose.pose.position.x;
	array_pose[1] = pose.pose.position.y;
	array_pose[2] = pose.pose.position.z;
	array_pose[3] = pose.pose.orientation.x;
	array_pose[4] = pose.pose.orientation.y;
	array_pose[5] = pose.pose.orientation.z;
	array_pose[6] = pose.pose.orientation.w;

}

void createLinearPullPoses(const geometry_msgs::PoseStamped &start_point, std::vector<geometry_msgs::Pose>& out_poses, int noOfPoses, std::vector<double> dir){


	out_poses.resize(noOfPoses+1);


	//Eigen::Translation3d rotation_center (radius*Eigen::Vector3d::UnitY());
	//compute the rotation center
	out_poses[0] = start_point.pose;

	double x_increment = dir[0] / noOfPoses;
	double y_increment = dir[1] / noOfPoses;
	double z_increment = dir[2] / noOfPoses;

	for (int i = 1; i <= noOfPoses; ++i)
	{
		geometry_msgs::Pose pose;
		pose.orientation = start_point.pose.orientation;
		pose.position.x = start_point.pose.position.x + x_increment * i;
		pose.position.y = start_point.pose.position.y + y_increment * i;
		pose.position.z = start_point.pose.position.z + z_increment * i;
		out_poses[i] = pose;
	}

}

void try_pull(moveit::planning_interface::MoveGroup *group, LearnKinematics learn, std::vector<double> dir){

	geometry_msgs::PoseStamped start_pose = learn.get_eef_pose();
	std::vector<geometry_msgs::Pose> out_poses;
	createLinearPullPoses(start_pose, out_poses, 4, dir);
	
	for (int i = 1; i<out_poses.size(); i++){
		geometry_msgs::PoseStamped interm_pose;
		interm_pose.header.frame_id ="/odom_combined";
		interm_pose.header.stamp = ros::Time::now();
		interm_pose.pose = out_poses.at(i);
		visualization_msgs::InteractiveMarker gripper = makeGripperMarkerI("at "+i, interm_pose);
		//server.insert(gripper);
		learn.try_action(interm_pose.pose, group);
		sleep(2);
		//server.applyChanges();
	}
	learn.try_action(start_pose.pose, group);
	

}

void try_open(moveit::planning_interface::MoveGroup *group, LearnKinematics learn, double angle, double radius, double waypoints){
	//TODO bug in here that orients gripper in a way not desired
	geometry_msgs::PoseStamped start_pose = learn.get_eef_pose();

	std::vector<geometry_msgs::Pose> out_poses;
	createCircularArcPoses(start_pose, out_poses, angle, radius, waypoints, Eigen::Vector3d::UnitZ());
	for (int i = 1; i<out_poses.size(); i++){
		geometry_msgs::PoseStamped interm_pose;
		interm_pose.header.frame_id ="/odom_combined";
		interm_pose.header.stamp = ros::Time::now();
		interm_pose.pose = out_poses.at(i);
		visualization_msgs::InteractiveMarker gripper = makeGripperMarkerI("at "+i, interm_pose);
		//server.insert(gripper);
		learn.try_action(interm_pose.pose, group);
		sleep(2);
		//server.applyChanges();
	}
	learn.try_action(start_pose.pose, group);

}

void enable_hand(bool is_left, moveit::planning_interface::MoveGroup *group){
	if(is_left)
	{
		group = new moveit::planning_interface::MoveGroup("left_arm");
		ROS_INFO_STREAM("Left Arm selected" );
	}
	else
	{
		group = new moveit::planning_interface::MoveGroup("right_arm");
		ROS_INFO_STREAM("Right Arm selected" );
	}

}

int main(int argc, char** argv){

	ros::init(argc, argv, "LearnKinematics");
	ros::AsyncSpinner spinner(1);//use 4 threads
	spinner.start();

	LearnKinematics learn;
	learn.initialize();
	moveit::planning_interface::MoveGroup* group;
	bool is_left = false;
	if(is_left)
	{
		group = new moveit::planning_interface::MoveGroup("left_arm");
		ROS_INFO_STREAM("Left Arm selected" );
	}
	else
	{
		group = new moveit::planning_interface::MoveGroup("right_arm");
		ROS_INFO_STREAM("Right Arm selected" );
	}
	
	geometry_msgs::PoseStamped start_pose = learn.get_eef_pose();

	//try_open(group, learn, -1.57/4.0,0.4,4);
	//try_open(group, learn, 1.57/4.0,-0.4,4);
	std::vector<double> dir(3);
	dir[0] = 0;
	dir[1] = 0;
	dir[2] = 0.3;
	try_pull(group, learn, dir);
	sleep(2);

	return 0;
}
