/*
   Gripper gripper;
   gripper.open();

   geometry_msgs::PoseStamped start = learn.get_eef_pose();
   populate_coords(start_pose, start);
   populate_coords(prev_pose, start);

   learn.try_action(-0.15, group);
   double diff = 1000;

   sleep(1);
   int i = 0;
   while(true){
   geometry_msgs::PoseStamped pose = learn.get_eef_pose();
   populate_coords(current_pose, pose);        


   poses[i][0] = i;
   poses[i][1] =pose.pose.position.x;
   poses[i][2] =pose.pose.position.y;
   poses[i][3] =pose.pose.position.z;
   poses[i][4] =pose.pose.orientation.x;
   poses[i][5] =pose.pose.orientation.y;
   poses[i][6] =pose.pose.orientation.z;
   poses[i][7] =pose.pose.orientation.w;

   ROS_INFO("Moving...");

   if (find_diff(prev_pose,current_pose) < 0.01){
   ROS_INFO("PR2 is at rest.");
   break;
   } 
   copy_over(prev_pose,current_pose,7);
   i++;
   }

   double distance = find_diff(start_pose, current_pose);

   ROS_INFO_STREAM("Distance traveled is " << distance);

   sleep(5.0);
   learn.try_action(0.15, group);
 */



        /*
           for(int i = 0; i < 8; i++){
        //std_msgs::String str;
        //std::ostringstream str_pose;
        geometry_msgs::PoseStamped pose = learn.get_eef_pose();

        poses[i+iterations][0] = i;
        poses[i+iterations][1] =pose.pose.position.x;
        poses[i+iterations][2] =pose.pose.position.y;
        poses[i+iterations][3] =pose.pose.position.z;
        poses[i+iterations][4] =pose.pose.orientation.x;
        poses[i+iterations][5] =pose.pose.orientation.y;
        poses[i+iterations][6] =pose.pose.orientation.z;
        poses[i+iterations][7] =pose.pose.orientation.w;

        //file << "i: " << i << std::endl 
        //     << "Position - x: "<< pose.pose.position.x << " y: " 
        //     << pose.pose.position.y << " z: " << pose.pose.position.z 
        //     << std::endl << "Orientation - x: " << pose.pose.orientation.x 
        //     << " y: " << pose.pose.orientation.y << " z: " << pose.pose.orientation.z 
        //     << " w: " << pose.pose.orientation.w << std::endl << std::endl;

        //    str.data = str_pose.str();
        //    bag.write("pose 2", ros::Time::now(), str);
        //ROS_INFO_STREAM(pose);
        //sleep(0.05);
        }
         */
        /*
           int movement = 1;
           for(int i = 0; i < (sizeof(poses)/sizeof(*poses)); i++){
           if(i % iterations == 0){
           file << "Movement " << movement << std::endl;
           movement++;
           }

           file << "i: "<< poses[i][0] << std::endl 
           << "Position - x: " << poses[i][1] << " y: " << poses[i][2] << " z: " << poses[i][3] << std::endl 
           << "Orientation - x" << poses[i][4] << " y: "<< poses[i][5]<< " z: " << poses[i][6] << " w: " << poses[i][7]<< std::endl<<std::endl;
           }

           ROS_INFO("movement 2 made");
         */


        /*
           sleep(3.0);

           gripper.findTwoContacts();
           gripper.slipServo();

        //sleep(2.0);//TODO move up here

        learn.try_action(-0.12, group);


        gripper.place();

        group->stop();
        gripper.open();
         */  

        //gripper.hold(10.0);

        /*
           gripper_clientr_ = new GripperClient("r_gripper_controller/gripper_action", true);
           while(!gripper_clientr_->waitForServer(ros::Duration(5.0))){
           ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
           }
         */

        //learn.get_width(gripper_clientr_, 0.001);
        //learn.open(gripper_clientr_, 0.0, 25.0);
        //learn.try_action(-0.15); //5 cm displacement




        //visualization_msgs::InteractiveMarker gripper_marker = makeGripperMarker("start_point", start_pose);
        //server.insert(gripper_marker);




        //geometry_msgs::PoseStamped center_pose;
        //center_pose.header.frame_id ="/odom_combined";
        //center_pose.header.stamp = ros::Time::now();
        //center_pose.pose = out_poses.at(0);
        //visualization_msgs::InteractiveMarker center_marker = makeQRIntMarkerCircle("center", center_pose);
        //server.insert(center_marker);


        //server.applyChanges();


        //bag.close();
        //file.close();










        //rosbag::Bag bag;
        //std::ostringstream filename;
        //how many values we want to display for a movement


        //interactive_markers::InteractiveMarkerServer server("marker");

        //boost::posix_time::ptime posix_time = ros::Time::now().toBoost();
        //std::string time_str = boost::posix_time::to_iso_extended_string(posix_time);  

        //filename << "coords_" << time_str << ".txt";
        //bag.open(filename.str(), rosbag::bagmode::Write);
        //std::ofstream file;
        //file.open(filename.str().c_str()); 
