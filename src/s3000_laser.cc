/*********************************************************************
*
*  This program uses the sicks3000  component to get laser scans, and then
*  publishes them as ROS messages
*
*  Copyright (c) 2012, Robotnik Automation, SLL
*
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#include <assert.h>
#include <math.h>
#include <iostream>
#include <boost/format.hpp>

#include "sicks3000.h"   // s3000 driver from player (Toby Collet / Andrew Howard)
#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"

using namespace std;

class s3000node {

public:
  SickS3000 *laser;
  sensor_msgs::LaserScan reading;

  string port_;

  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher laser_data_pub_;

  //for diagnostics
  bool connected_;
  bool getting_data_;

  string frameid_;

  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;


  s3000node(ros::NodeHandle h) : self_test_(), diagnostic_(),
    node_handle_(h), private_node_handle_("~"),
    desired_freq_(20),
    connected_(false),
    getting_data_(false),
    freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
  {
    ros::NodeHandle laser_node_handle(node_handle_, "s3000_laser");

    private_node_handle_.param("port", port_, string("/dev/ttyUSB0"));
    private_node_handle_.param("frame_id", frameid_, string("laser"));
    reading.header.frame_id = frameid_;

    laser_data_pub_ = laser_node_handle.advertise<sensor_msgs::LaserScan>("scan", 100);

    self_test_.add( "Connect Test", this, &s3000node::ConnectTest );
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( "Laser S3000 Status", this, &s3000node::deviceStatus );

    // Create SickS3000 in the given port
    laser = new SickS3000( port_ );
   }


  ~s3000node()
  {
    stop();
  }


  int start()
  {
    stop();

    if (laser->Open() == 0)
    {
      diagnostic_.setHardwareID("Laser Ranger");
      ROS_INFO("Laser Ranger sensor initialized.");
      freq_diag_.clear();
      connected_ = true;
    }
    else
    {
      ROS_ERROR("Laser Ranger sensor initialization failed.");
      connected_ = false;
      diagnostic_.force_update();
      return (-1);
    }

    return(0);
  }


  int stop()
  {
    if(connected_)
    {
        laser->Close();
        connected_ = false;
    }

    return(0);
  }


  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (laser->Open() == 0)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected successfully.");
    }
    else
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Connection unsuccessful.");
    }
  }


  int getData(sensor_msgs::LaserScan& data)
  {
    bool bValidData = false;

    if (laser->ReadLaser( data, bValidData ) < 0)
    {
      return (-1);
    }

    //// If valid data, publish it
    if (bValidData)
    {
      data.header.stamp = ros::Time::now();
      laser_data_pub_.publish( data );
      freq_diag_.tick();
      getting_data_ = true;
    }
    else
    {
      getting_data_ = false;
    }

    return (0);
  }


  bool spin()
  {
    ros::Rate r(desired_freq_);

    // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    while (!ros::isShuttingDown())
    {
      if (start() == 0)
      {
        while(node_handle_.ok())
        {
          if (getData(reading) == -1) //Error reading from port
          {
            connected_ = false;
          }
          else
          {
            connected_ = true;
          }

          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
          r.sleep();
        }
      }
      else
      {
        // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        usleep(1000000);
        self_test_.checkTest();
        ros::spinOnce();
      }
    }

    stop();

    return true;
  }


  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (!connected_)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Lidar is not connected");
    }
    else if (!getting_data_)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No valid data from lidar");
    }
    else
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Lidar is running");
    }

    status.add("Device", port_);
    status.add("TF frame", frameid_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "s3000_node");

  ros::NodeHandle nh;

  s3000node s3000n(nh);
  s3000n.spin();

  return(0);
}
