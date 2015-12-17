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
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "diagnostic_updater/update_functions.h"

#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"

using namespace std;

class s3000node {

public:
  boost::scoped_ptr<SickS3000> laser_;
  sensor_msgs::LaserScan scan_msg_;

  string port_;

  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  //for diagnostics
  bool connected_;
  bool getting_data_;

  string frameid_;

  double desired_freq_;

  typedef diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> LaserScanDiagnosedPublisher;
  boost::scoped_ptr<LaserScanDiagnosedPublisher> data_pub_;

  s3000node(ros::NodeHandle h) :
    node_handle_(h), private_node_handle_("~"),
    desired_freq_(16.7),
    connected_(false),
    getting_data_(false)
  {
    private_node_handle_.param("port", port_, string("/dev/ttyUSB0"));
    private_node_handle_.param("frame_id", frameid_, string("laser"));
    scan_msg_.header.frame_id = frameid_;

    self_test_.add( "Connect Test", this, &s3000node::ConnectTest );

    diagnostic_.add( "Laser S3000 Connection", this, &s3000node::deviceStatus );

    data_pub_.reset(new LaserScanDiagnosedPublisher(
          node_handle_.advertise<sensor_msgs::LaserScan>("scan", 1), diagnostic_,
          diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05),
          diagnostic_updater::TimeStampStatusParam()));

    laser_.reset(new SickS3000(port_));
   }


  ~s3000node()
  {
    stop();
  }


  bool start()
  {
    stop();

    if (laser_->Open() == 0)
    {
      diagnostic_.setHardwareID("Laser Ranger");
      ROS_INFO("Laser Ranger sensor initialized.");
      connected_ = true;
      return true;
    }
    else
    {
      ROS_ERROR("Laser Ranger sensor initialization failed.");
      connected_ = false;
      diagnostic_.force_update();
      return false;
    }
  }


  void stop()
  {
    if(connected_)
    {
        laser_->Close();
        connected_ = false;
    }
  }


  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (laser_->Open() == 0)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected successfully.");
    }
    else
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Connection unsuccessful.");
    }
  }

  bool spin()
  {
    // Outer loop for reconnection.
    while (ros::ok())
    {
      diagnostic_.update();
      ros::spinOnce();

      if (start())
      {
        // Inner loop runs once per received scan.
        while(ros::ok())
        {
          bool scan_available = false;
          if (laser_->ReadLaser(scan_msg_, scan_available) < 0)
          {
            // Problem reading, or read timed out. Disconnect and try again.
            connected_ = false;
            getting_data_ = false;
            ros::Duration(1.0).sleep();
            break;
          }

          if (scan_available)
          {
            data_pub_->publish(scan_msg_);
            getting_data_ = true;
          }
          else
          {
            getting_data_ = false;
          }

          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
        }
      }
      else
      {
        ROS_INFO("Retrying in 1 second.");
        ros::Duration(1.0).sleep();
      }
    }

    stop();

    return true;
  }


  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (!connected_)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "LIDAR not connected.");
    }
    else if (!getting_data_)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "LIDAR connected, but no data.");
    }
    else
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "LIDAR connected.");
    }

    status.add("Device", port_);
    status.add("TF Frame", frameid_);
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
