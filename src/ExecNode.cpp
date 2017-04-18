/*
 * Copyright (C) 2017, Elad Denenberg.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Technion or Elad Denenberg. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "tum_executioner/ExecMsg.h"
#include "tum_executioner/ExParamsConfig.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

//declarations

std::string mapname = "map.txt"; //change this to fit your map filename

//functions

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
	tum_exc_com_ = nex_.advertise<std_msgs::String>("/tum_ardrone/com", 1000);
	tum_exc_pub_ = nex_.advertise<tum_executioner::ExecMsg>("/ExecTop", 1000);
	tum_exc_sub_ = nex_.subscribe("/ExecTop", 1000, &SubscribeAndPublish::ExecCb,this);
	tum_exc_cvel_= nex_.subscribe("/cmd_vel",10, &SubscribeAndPublish::cvelCb,this);
	tum_exc_img_ = nex_.subscribe("/ardrone/image_raw",1, &SubscribeAndPublish::imageCb, this);
	
    }
    int building_ = 0;
    int points_ = 0;
    int counter_ = 0;
    float xvel_ = 999, yvel_ = 999, zvel_ = 999, wvel_ = 999;
    int number_of_buildings_,number_of_points_;
    cv_bridge::CvImagePtr cv_ptr;

    void cvelCb(geometry_msgs::Twist cvel_msg)
    {
     //counter++;
     if (counter_ > 10) {
          //ROS_INFO("got here");
          xvel_ = cvel_msg.linear.x;
          yvel_ = cvel_msg.linear.y;
          zvel_ = cvel_msg.linear.z;
	  wvel_ = cvel_msg.angular.z;

     } //endif
    } //endCb

            
        
    void imageCb(const sensor_msgs::ImageConstPtr& img)
	{ 
	 cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}

    void ExecCb(const tum_executioner::ExecMsg msg)
    {
       tum_executioner::ExecMsg pub_msg = msg;
       if ((building_!=msg.building) || (points_ != msg.building_point))
       {
          building_ = msg.building;
          points_ = msg.building_point;
       
	 //ROS_INFO("building_ = %d , msg.building = %d , points_ = %d , msg.points_ = %d ", building_,msg.building,points_, msg.building_point);
  	 std::string command = "c goto ";  
         gotopoint = read_map(mapname,building_, points_);

         command += gotopoint;
  
         std_msgs::String com_msg;
         com_msg.data = command; // the command now contains "goto -whatever was in the map*"  i.e. "goto 1 2 1 90" 
   
         tum_exc_com_.publish(com_msg); //send the command to the autopilot

         
	 
         while ( ((xvel_ * xvel_) > 0.005) || 
		 ((yvel_ * yvel_) > 0.005) || 
		 ((zvel_ * zvel_) > 0.005) ||
		 ((wvel_ * wvel_) > 0.005) ) //wait until the you care close enough to the target. "/cmd_vel" is a vector containing the command velocity, and during flight is usually around 0.22 for 
				             //each direction, when in rest it is usually around 0.02 
         {
            
            ROS_INFO("x %f, y %f z %f w %f ",xvel_,yvel_ ,zvel_,wvel_ );
            counter_++;
	    ros::spinOnce();
	    
             
         }//endwhile
	 						// TODO save image file !!!!!!!!!!!!!
	 ROS_INFO("x %f, y %f z %f w %f ",xvel_,yvel_ ,zvel_,wvel_ );
         ROS_INFO("counter = %d ", counter_);
	 std::remove("Image.png");
	 cv::imwrite("Image.png",cv_ptr->image);
         pub_msg.fileready = true;
	 tum_exc_pub_.publish(pub_msg);
	 
	
         counter_ = 0;
	 xvel_ = 999; yvel_ = 999; zvel_ = 999; wvel_ = 999;
	
      }//endif
      
    }//endCb

    std::string read_map(const std::string mapname,int building,int points)
    { 
      std::string gotopoint;
      if ((building <= number_of_buildings_) && (points <= number_of_points_))
      {
        int gotoline = (building - 1)*number_of_points_ + points;
        std::string mapfilename = ros::package::getPath("tum_executioner");
	mapfilename += "/"+mapname;
        mapFile.open(mapfilename.c_str(), std::ios::app);
        if (mapFile.is_open())
        {
          //std::cout << "File is open."<<std::endl;
          for (int i =1; i <= gotoline; i++)
          {
            std::getline(mapFile, gotopoint);
          }//endfor
          mapFile.close(); 
        }//endif
      }//endif
      return(gotopoint);
    }//endfunc

  //void dynConfCb(tum_executioner::ExParamsConfig &config, uint32_t level)
  //{
  // number_of_buildings_ = config.number_of_buildings;
  //  number_of_points_ = config.number_of_points;
  //}
    
  inline void reset(void)
    {
     ROS_INFO("reseting");

     //tum_executioner::ExParamsConfig config;  //TODO Fix dynamic parameters.
     number_of_buildings_ = 1;//config.number_of_buildings;
     number_of_points_ = 4;//config.number_of_points;
     //ROS_INFO("%d , %d",config.number_of_buildings_,config.number_of_points_);
     tum_executioner::ExecMsg reset_msg; // reset the points on the map
     reset_msg.building = 0;
     reset_msg.building_point = 0;
     reset_msg.fileready = false;     

     	
     std::string  command;
     std_msgs::String cmd_msg;	
     command = "c start";
     cmd_msg.data = command;
     ros::Rate poll_rate(100);
     while(tum_exc_com_.getNumSubscribers() == 0)
        poll_rate.sleep();
     tum_exc_com_.publish(cmd_msg);
     cmd_msg.data = "c takeoff";
     tum_exc_com_.publish(cmd_msg);
     while(tum_exc_pub_.getNumSubscribers() == 0)
        poll_rate.sleep();
     tum_exc_pub_.publish(reset_msg);
     
    }//endfunc
    
private:
    ros::NodeHandle nex_;
    ros::Publisher  tum_exc_com_;
    ros::Publisher  tum_exc_pub_;
    ros::Subscriber tum_exc_sub_;
    ros::Subscriber tum_exc_cvel_;
    ros::Subscriber tum_exc_img_;

    std::ifstream mapFile;
    std::string gotopoint;
    geometry_msgs::Twist cmd_vel;

};// End of class


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ExecNode");

	SubscribeAndPublish SubNPub;
	ROS_INFO("starting");
	SubNPub.reset();
	

	// loop
	ros::Rate loop_rate(10);
  
  	int count = 0;
  

 	while (ros::ok())
  	{
		// 
		ros::spinOnce();

    		loop_rate.sleep();
    		++count;
	}
    
	

}
