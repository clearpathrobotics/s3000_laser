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
#include <boost/scoped_ptr.hpp>

#include "sicks3000.h"   // s3000 driver from player (Toby Collet / Andrew Howard)
#include "ros/time.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "diagnostic_updater/update_functions.h"

#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"

#define RADIANS(X) ((X) * 3.141592653589793 / 180.0)


using namespace std;

class s3000node {

public:
  boost::scoped_ptr<SickS3000> laser_;
  sensor_msgs::LaserScan scan_msg_;

  string port_;

  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  enum {
    STATE_DISCONNECTED,
    STATE_CONNECTED,
    STATE_RESTARTING,
    STATE_ERROR_SCANS,
    STATE_OK
  } state_;

  bool host_lidar_;
  bool enable_scid_error_;
  int expected_scid_;
  uint16_t scid_;
  bool tx_failure_;
  uint8_t seven_seg_first_char_;
  uint8_t seven_seg_second_char_;
  ros::Time error_state_start_;

  /**
   * List of error codes which we are permitted to restart the LIDAR out of.
   */
  ros::V_string restart_errors_;

  /**
   * Times we have restarted without a 60 second period of normal operation
   * in between.
   */
  int successive_restarts_;

  /**
   * When this limit is hit, no more restarting. This prevents us from cycling
   * the LIDAR when it is doing no good.
   */
  int successive_restart_limit_;

  /**
   * Tracks errors detected for the lifetime of the node; the string is an
   * error_code value, and the int is a counter.
   */
  typedef std::map<std::string, int> ErrorCountersMap;
  ErrorCountersMap error_counters_;

  string frameid_;

  double desired_freq_;

  double range_max_;  // [m]
  double range_min_;  // [m]

  typedef diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> LaserScanDiagnosedPublisher;
  boost::scoped_ptr<LaserScanDiagnosedPublisher> data_pub_;

  s3000node(ros::NodeHandle h) :
    node_handle_(h), private_node_handle_("~"),
    state_(STATE_DISCONNECTED),
    successive_restarts_(0),
    successive_restart_limit_(10),
    desired_freq_(16.6),
    range_max_(51.0),  // NOTE datasheet reports 52.0m, but actual is 51.2m
    range_min_(0.051)   // Determined by observation.
  {
    private_node_handle_.param("port", port_, string("/dev/ttyUSB0"));
    private_node_handle_.param("frame_id", frameid_, string("laser"));
    private_node_handle_.param("range_max", range_max_, range_max_);
    private_node_handle_.param("range_min", range_min_, range_min_);
    private_node_handle_.param("host_lidar", host_lidar_, true);
    private_node_handle_.param("expected_scid", expected_scid_, -1);
    private_node_handle_.param("enable_scid_error", enable_scid_error_, true);
    private_node_handle_.param("successive_restart_limit", successive_restart_limit_, successive_restart_limit_);

    if (private_node_handle_.hasParam("restart_errors"))
    {
      private_node_handle_.getParam("restart_errors", restart_errors_);
    }


    double desired_freq_tolerance = 0.2;
    private_node_handle_.param("desired_frequency", desired_freq_, desired_freq_);
    private_node_handle_.param("desired_frequency_tolerance", desired_freq_tolerance, desired_freq_tolerance);

    scan_msg_.header.frame_id = frameid_;
    scan_msg_.angle_min = static_cast<float>(RADIANS(-95.0));
    scan_msg_.angle_max = static_cast<float>(RADIANS(95.0));
    scan_msg_.angle_increment = static_cast<float>(RADIANS(0.25));
    scan_msg_.range_min = range_min_;
    scan_msg_.range_max = range_max_;

    diagnostic_.add("connection status", this, &s3000node::deviceStatusDiagnostics);

    diagnostic_.add("errors", this, &s3000node::errorLogDiagnostics);

    data_pub_.reset(new LaserScanDiagnosedPublisher(
          node_handle_.advertise<sensor_msgs::LaserScan>("scan", 1), diagnostic_,
          diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, desired_freq_tolerance),
          diagnostic_updater::TimeStampStatusParam()));

    laser_.reset(new SickS3000(port_));

    scid_ = 0x0000;
    seven_seg_first_char_ = 0x00;
    seven_seg_second_char_ = 0x00;
    tx_failure_ = false;
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
      state_ = STATE_CONNECTED;
      return true;
    }
    else
    {
      ROS_ERROR_THROTTLE(10.0, "Laser Ranger sensor initialization failed.");
      ROS_DEBUG("Laser Ranger sensor initialization failed.");
      state_ = STATE_DISCONNECTED;
      diagnostic_.force_update();
      return false;
    }
  }


  void stop()
  {
    if (state_ >= STATE_CONNECTED)
    {
      laser_->Close();
      state_ = STATE_DISCONNECTED;
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
          laser_->WaitForScan(scan_msg_);
          ros::Duration(0.04).sleep();

          bool scan_available = false;
          if (laser_->ReadLaser(scan_msg_, scan_available) < 0)
          {
            if (state_ != STATE_RESTARTING)
            {
              // Problem reading, or read timed out. Disconnect and try again.
              state_ = STATE_DISCONNECTED;
              ros::Duration(3.0).sleep();
              break;
            }
          }

          if (scan_available)
          {
            if (scan_msg_.ranges[0] > range_min_)
            {
              // Looks like a valid scan.
              state_ = STATE_OK;
              scan_msg_.scan_time = (1.0 / desired_freq_);
              scan_msg_.time_increment = scan_msg_.scan_time / scan_msg_.ranges.size();
              data_pub_->publish(scan_msg_);

              if (ros::Time::now() - error_state_start_ > ros::Duration(60.0) && successive_restarts_ > 0)
              {
                // We are receiving valid scans, and it has been 60 seconds since our
                // most recent error -> reset the counter.
                successive_restarts_ = 0;
                ROS_DEBUG("successive_restarts = %d", successive_restarts_);
              }
            }
            else
            {
              // When in an error state, the LIDAR reports all minimum range values.
              // Setting the state here flags the diagnostics function to query the LIDAR
              // for status code, and optionally reset it.
              ROS_DEBUG("LIDAR scan values indicate scanner error.");
              state_ = STATE_ERROR_SCANS;
            }
          }

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

  void errorLogDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    int total = 0;

    ErrorCountersMap::iterator it;
    for (it = error_counters_.begin(); it != error_counters_.end(); ++it)
    {
      total += it->second;
      std::stringstream ss;
      ss << "Error count for code [" << it->first << "]";
      status.add(ss.str(), it->second);
    }

    if (total == 0)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "No LIDAR errors seen.");
    }
    else
    {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "%d LIDAR error(s) seen.", total);
    }
  }

  void deviceStatusDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    switch (state_)
    {
      case STATE_DISCONNECTED:
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "LIDAR not connected.");
        break;
      case STATE_CONNECTED:
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "LIDAR connected, not receiving scans.");
        break;
      case STATE_RESTARTING:
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "LIDAR commanded to restart.");
        break;
      case STATE_ERROR_SCANS:
        // Delegate this case to a separate function.
        errorDiagnostics(status);
        break;
      case STATE_OK:
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "LIDAR OK.");
        if (scid_ == 0)
        {
          // If we just started up and don't yet know our SCID, trigger the error diagnostics one time
          // to find out.
          errorDiagnostics(status);
        }
        break;
    }


    if ((expected_scid_ != scid_) && (expected_scid_ != -1) && !tx_failure_)
    {
      status.summary(enable_scid_error_ ? (int)diagnostic_msgs::DiagnosticStatus::ERROR :
                                          (int)diagnostic_msgs::DiagnosticStatus::WARN,
                     "LIDAR configuration has invalid SCID.");
    }

    status.add("Device", port_);
    status.add("TF Frame", frameid_);
    status.add("Range max", range_max_);

    status.addf("SCID", "0x%04X", scid_);

    if (expected_scid_ != -1)
    {
      status.addf("Expected SCID", "0x%04X", expected_scid_);
    }
    else
    {
      status.add("Expected SCID", "Unset");
    }
  }

  void errorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    bool first_time = false;

    if (tx_failure_)
    {
      // Early return when we've had a previous communications failure.
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
          "LIDAR cannot be commanded; check comms interface.");
      return;
    }

    if (seven_seg_first_char_ == 0 && seven_seg_second_char_ == 0)
    {
      // Query for error code, since we don't know it yet. On successive calls,
      // assume it has not changed.
      first_time = true;
      if (laser_->getDiagnosticInfo(host_lidar_, &scid_,
            &seven_seg_first_char_, &seven_seg_second_char_) < 0)
      {
        // Unable to interact with the device. Mark us as having a communications
        // failure so that we don't keep on re-attempting it.
        tx_failure_ = true;
      }
    }

    // Only do the stuff below if there's an actual error. There might not be,
    // since we also trigger this check on device startup.
    if (seven_seg_first_char_ != 0 || seven_seg_second_char_ != 0)
    {
      std::string error_code;
      std::string error_description;
      std::string error_suggestion;
      laser_->GetDeviceError(seven_seg_first_char_, seven_seg_second_char_, &error_code,
                             &error_description, &error_suggestion);

      if (first_time)
      {
        error_state_start_ = ros::Time::now();
        successive_restarts_++;
        ROS_DEBUG("successive_restarts = %d", successive_restarts_);

        if (error_counters_.count(error_code) == 0)
        {
          error_counters_[error_code] = 0;
        }
        error_counters_[error_code]++;
      }

      status.add("Error Code", error_code);
      status.add("Error Description", error_description);
      status.add("Suggested Actions", error_suggestion);
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "LIDAR error (%s)", error_code.c_str());

      // There is a race condition where we can sometimes end up catching the LIDAR
      // showing its startup code. This logic will suppress that and cause us to poll
      // its error code again next time into this function.
      if (error_code == "H" || error_code == "G" || error_code == "?" || error_code == "")
      {
        ROS_WARN("Suppressing error code in order to re-check.");
        seven_seg_first_char_ = 0;
        seven_seg_second_char_ = 0;
      }

      if (host_lidar_ && canRestartError(error_code) &&
          successive_restarts_ < successive_restart_limit_ &&
          (ros::Time::now() - error_state_start_ > ros::Duration(2.0)))
      {
        ROS_INFO("Triggering LIDAR restart to attempt to clear error.");
        laser_->triggerRestart();

        // Clear the stored error code so that we know to query it again if there's
        // another error within the lifespan of this node.
        seven_seg_first_char_ = 0;
        seven_seg_second_char_ = 0;
        state_ = STATE_RESTARTING;
      }
    }
  }

  bool canRestartError(std::string error_code)
  {
    for (int i = 0; i < restart_errors_.size(); i++)
    {
      if (error_code == restart_errors_[i])
      {
        return true;
      }
    }
    return false;
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
