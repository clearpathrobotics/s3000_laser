/*
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
*
*/
/*
 Desc: Driver for the SICK S3000 laser
 Author: Robotnik Automation SLL (based on sicks3000 by Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard for Player/Stage)
 Date: 1 Sept 2012

 The sicks3000 driver controls the SICK S 3000 safety laser scanner interpreting its data output.
 The driver is very basic and assumes the S3000 has already been configured to continuously output
 its measured data on the RS422 data lines.
*/

#include <netinet/in.h>  // htons
#include "sicks3000.h"

// 1 second of data at 500kbaud
#define DEFAULT_RX_BUFFER_SIZE 500*1024/8

// Device codes
#define STX     0x02
#define ACK     0xA0
#define NACK    0x92
#define CRC16_GEN_POL 0x8005

/*! \fn SickS3000::SickS3000()
 *  \brief Public constructor
*/
SickS3000::SickS3000(std::string port)
{
  rx_count = 0;
  rx_buffer_size = DEFAULT_RX_BUFFER_SIZE;
  rx_buffer = new uint8_t[rx_buffer_size];
  assert(rx_buffer);

  recognisedScanner = false;
  mirror = 0;

  serial = new SerialDevice(port.c_str(), S3000_DEFAULT_TRANSFERRATE, S3000_DEFAULT_PARITY, S3000_DEFAULT_DATA_SIZE); //Creates serial device

  return;
}

SickS3000::~SickS3000()
{
  // Close serial port
  if (serial!=NULL) serial->ClosePort();

  // Delete serial port
  if (serial!=NULL) delete serial;

  delete [] rx_buffer;
}

/*! \fn int SickS3000::Open()
 *  \brief Open serial port
 *  \returns -1 Error
 *  \returns 0 Ok
*/
int SickS3000::Open()
{
  // Setup serial device
  if (this->serial->OpenPort2() == SERIAL_ERROR)
  {
    ROS_ERROR_ONCE("SickS3000::Open: Error Opening Serial Port");
    ROS_DEBUG("SickS3000::Open: Error Opening Serial Port");
    return -1;
  }

  ROS_INFO("SickS3000::Open: serial port opened at %s", serial->GetDevice());

  return 0;
}

/*! \fn int SickS3000::Close()
 *  \brief Closes serial port
 *  \returns ERROR
 *  \returns OK
*/
int SickS3000::Close()
{
  if (serial!=NULL) serial->ClosePort();
  return 0;
}

/*----------------------------------------------------------------------------
** Laser Scanning
*---------------------------------------------------------------------------*/
int SickS3000::WaitForScan(sensor_msgs::LaserScan& scan)
{
  // Wait up to 100ms for new data to be available.
  if (!serial->BlockOnRead(100))
  {
    ROS_ERROR_THROTTLE(10.0, "SickS3000::ReadLaser: Error waiting to read port.");
    ROS_DEBUG("SickS3000::ReadLaser: Error waiting to read port.");
    return -1;
  }

  // Data is flowing, get our timestamp.
  scan.header.stamp = ros::Time::now();
  
  return 0;
}

int SickS3000::ReadLaser(sensor_msgs::LaserScan& scan, bool& bValidData)
{
  int read_bytes=0;     // Number of received bytes
  char cReadBuffer[2000] = "\0";    // Max in 1 read
  
  // Read controller messages.
  if (serial->ReadPort(cReadBuffer, &read_bytes, 1999) < 0)
  {
    ROS_ERROR("Error reading from port.");
    return -1;
  }
  else if (read_bytes == 0)
  {
    ROS_ERROR("No data returned when reading from port.");
    return -1;
  }
  ROS_DEBUG("RX %d bytes", read_bytes);

  unsigned int messageOffset = rx_count;
  rx_count += read_bytes;
  if (rx_count > rx_buffer_size)
  {
     ROS_WARN("S3000 Buffer Full");
     rx_count = 0;
  }
  else
  {
     memcpy(&rx_buffer[messageOffset], cReadBuffer, read_bytes);
     ProcessLaserData(scan, bValidData);
  }

  return 0;
}

int SickS3000::ProcessLaserData(sensor_msgs::LaserScan& scan, bool& bValidData)
{
  while(rx_count >= 22)
  {
    // find our continuous data header
    unsigned int ii;
    bool found = false;
    for (ii = 0; ii < rx_count - 22; ++ii)
    {
      if (memcmp(&rx_buffer[ii],"\0\0\0\0\0\0",6) == 0)
      {
        memmove(rx_buffer, &rx_buffer[ii], rx_count-ii);
        rx_count -= ii;
        found = true;
        break;
      }
    }

    if (!found)
    {
      memmove(rx_buffer, &rx_buffer[ii], rx_count-ii);
      rx_count -= ii;
      return 0;
    }

    // get relevant bits of the header
    // size includes all data from the data block number
    // through to the end of the packet including the checksum
    unsigned short size = 2*htons(*reinterpret_cast<unsigned short *> (&rx_buffer[6]));

    if (size > rx_buffer_size - 26)
    {
      ROS_WARN("S3000: Requested Size of data is larger than the buffer size");
      memmove(rx_buffer, &rx_buffer[1], --rx_count);
      return 0;
    }

    // check if we have enough data yet
    if (size > rx_count - 4)
      return 0;

    unsigned short packet_checksum = *reinterpret_cast<unsigned short *> (&rx_buffer[size+2]);
    unsigned short calc_checksum = CreateCRC(&rx_buffer[4], size-2);
    if (packet_checksum != calc_checksum)
    {
      ROS_WARN("S3000: Checksum's dont match, that's bad (data packet size %d)",size);
      memmove(rx_buffer, &rx_buffer[1], --rx_count);
      continue;
    }
    else
    {
      uint8_t * data = &rx_buffer[20];
      if (data[0] != data[1])
      {
        ROS_WARN("S3000: Bad type, header bytes dont match");
      }
      else
      {
        if (data[0] == 0xAA)
        {
          ROS_WARN("S3000: We got an I/O data packet and we don't know what to do with it\n");
        }
        else if (data[0] == 0xBB)
        {
          int data_count = (size - 22) / 2;
          if (data_count < 0)
          {
            ROS_WARN("S3000: bad data count (%d)", data_count);
            memmove(rx_buffer, &rx_buffer[size+4], rx_count - (size+4));
            rx_count -= (size + 4);
            continue;
          }

          // Scan data, clear ranges, keep configuration
          scan.ranges.clear();
          scan.intensities.clear();  // not used
          scan.ranges.resize(data_count);

          for (int ii = 0; ii < data_count; ++ii)
          {
            unsigned short Distance_CM = (*reinterpret_cast<unsigned short *> (&data[4 + 2*ii]));
            Distance_CM &= 0x1fff; // remove status bits
            double distance_m = static_cast<double>(Distance_CM)/100.0;

            if (mirror == 1)
              scan.ranges[data_count - ii - 1] = static_cast<float> (distance_m); // Reverse order.
            else
              scan.ranges[ii] = static_cast<float> (distance_m);
          }

          // CHECK
          // Return this flag to let the node know that the message is ready to publish
          bValidData = true;
        }
        else if (data[0] == 0xCC)
        {
          ROS_WARN("We got a reflector data packet and we don't know what to do with it\n");
        }
        else
        {
          ROS_WARN("We got an unknown packet\n");
        }
      }
    }

    memmove(rx_buffer, &rx_buffer[size+4], rx_count - (size+4));
    rx_count -= (size + 4);
    continue;
  }
  return 1;
}

/*----------------------------------------------------------------------------
** CRC
*---------------------------------------------------------------------------*/
static const unsigned short crc_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

unsigned short SickS3000::CreateCRC(uint8_t *Data, ssize_t length)
{
  unsigned short CRC_16 = 0xFFFF;
  unsigned short i;
  for (i = 0; i < length; i++)
  {
    CRC_16 = (CRC_16 << 8) ^ (crc_table[(CRC_16 >> 8) ^ (Data[i])]);
  }
  return CRC_16;
}

/*----------------------------------------------------------------------------
** Diagnostics
*---------------------------------------------------------------------------*/
#define SEG_0   0x00
#define SEG_1_  0xB0
#define SEG__1  0x86
#define SEG_2   0xDB
#define SEG_3   0xCF
#define SEG_4   0xE6
#define SEG_5   0xED
#define SEG_6   0xFD
#define SEG_7   0x87
#define SEG_8   0xFF
#define SEG_9   0xEF
#define SEG_d   0xDE
#define SEG_E   0xF9
#define SEG_F   0xF1
#define SEG_G   0xBD
#define SEG_H   0xF6
#define SEG_L   0xB8
#define SEG_n   0x54
#define SEG_o   0xDC
#define SEG_P   0xF3
#define SEG_u   0x9C
#define SEG_y   0xEE

typedef struct
{
  const uint8_t first_char;
  const uint8_t second_char;
  const char* cause;
  const char* suggested_action;

} ErrorCodeLUTEntry;

const ErrorCodeLUTEntry s3000_errors[] = {
  //Indication of protective field and contour infringements in the dual field mode
  {SEG_1_,SEG_0,"Object in protective field (1_.)","No error. The status indication eases system testing on the use of simultaneous protective fields or in an EFI system (if the OSSDs on the guest are not used, a protective field infringement is not signaled via the red LED as required in the standard)."},
  {SEG__1,SEG_0,"Object in the simultaneous protective field (_1.)","No error. The status indication eases system testing on the use of simultaneous protective fields or in an EFI system (if the OSSDs on the guest are not used, a protective field infringement is not signaled via the red LED as required in the standard)."},
  //Indication of protective field and contour infringements in the dual protective field mode
  {0x20,0x00,"Object in the first protective field of the field set","No error"},
  {0x10,0x00,"Object in the second protective field of the field set","No error"},
  {0x02,0x00,"Object in the first protective field of the simultaneous field set","No error"},
  {0x04,0x00,"Object in the second protective field of the simultaneous field set","No error"},
  //Indiation of protective field and warning field infringements in the triple field mode
  {0x01,0x00,"Object in protective field","No error"},
  {0x40,0x00,"Object in warning field 1","No error"},
  {0x08,0x00,"Object in warning field 2","No error"},
  //Shown for all field modes
  {SEG_3,SEG_0,"Initialization of the device or Waiting for the end of the initialization of a second device connected to the EFI (3.)","The display goes out automatically when the S3000 has been initialized and/or the connection to the second device has been made. If the display does not go off, check whether the partner device is in operation and/or check the wiring. If no partner device is connected, check the system configuration with the aid of the CDS and transfer the corrected configuration to the S3000 again if necessary."},
  {SEG_4,SEG_0,"Waiting for valid input signals (4.)","The indication extinguishes automatically if an input signal is present that corresponds to the configured evaluation type (1-of-n or complementary). If the display does not go off, check the wiring, check the control signals for correct switching behavior, check, if velocity ranges are used for monitoring case switching, whether the EFI status information Speed valid is transferred, check the system configuration with the aid of the CDS and transfer the corrected configuration to the S3000 again if necessary."},
  {SEG_6,SEG_0,"Waiting for configuration or configuration not completed (6.)","The display goes off automatically once the configuration has been successfully transferred. If the display does not go off, check the system configuration with the aid of the CDS and transfer the corrected configuration to the S3000 again if necessary, check whether the configuration saved in the system plug is compatible with the S3000."},
  {0x7D,0x7D,"Waiting for restart of the device (6 flashing)","Switch off the voltage supply for the S3000 for at least 2 seconds and then switch it back on."},
  {SEG_8,SEG_0,"Error of the external device monitoring (8.)","Check whether the contactors are working correctly or if they are wired incorrectly and rectify any error."},
  {SEG_8,SEG_8,"Error of the external device monitoring (8.)","If flashing, check whether the contactors are working correctly or if they are wired incorrectly and rectify any error, and additionally switch off the device and wait at least 3 seconds, then switch back on the power supply."},
  {SEG_9,SEG_0,"Error in the control switch for restart or reset (9.)","Check the functionality of the control switch. The button may be defective or stuck. Check the wiring of the control switch for short-circuit to 24 V."},
  {SEG_d,SEG__1,"Velocity tolerance exceeded: The difference between the velocities measured by the incremental encoders is too large. (d. - _1.)","Check the incremental encoders. Check the configuration of the incremental encoder inputs with the aid of the CDS."},
  {SEG_d,SEG_2,"Direction of movement output by the incremental encoders is different. (d. - 2.)","Check the wiring of the incremental encoder inputs, e.g. for incorrect pin assignments."},
  {SEG_d,SEG_3,"Maximum frequency at input INC1 exceeded (d. - 3.)","Check the incremental encoders. Check the configuration of the incremental encoder inputs with the aid of the CDS. Check whether the permitted maximum velocity of the vehicle is exceeded!,"},
  {SEG_d,SEG_4,"Maximum frequency at input INC2 exceeded or The monitored speed threshold has been exceeded.(d. - 4.)","Check the incremental encoders. Check the configuration of the incremental encoder inputs with the aid of the CDS. Check whether the permitted maximum velocity of the vehicle is exceeded! Check the limit velocity configured in the related monitoring cases."},
  {SEG_E,SEG__1,"Sensor head faulty (E. - _1.)","Switch off the voltage supply of the S3000 for at least 2 s and then switch it back on. If the display does not go off: Send the sensor head, the I/O module or the system plug to the manufacturer for repair."},
  {SEG_E,SEG_2,"I/O module faulty (E. - 2.)","Switch off the voltage supply of the S3000 for at least 2 s and then switch it back on. If the display does not go off: Send the sensor head, the I/O module or the system plug to the manufacturer for repair."},
  {SEG_E,SEG_3,"Configuration memory in the system plug faulty (E. - 3.)","Switch off the voltage supply of the S3000 for at least 2 s and then switch it back on. If the display does not go off: Send the sensor head, the I/O module or the system plug to the manufacturer for repair."},
  {SEG_E,SEG_4,"A second device which is connected via EFI is faulty. (E. - 4.)","Check the connected device and theconnection."},
  {SEG_F,SEG__1,"Overcurrent on OSSD connection 1 (F. - _1.)","Check the switching element connected (contactor, relay). Replace, if necessary. Check the wiring for short-circuit to 0 V."},
  {SEG_F,SEG_2,"Short-circuit to 24 V at OSSD connection 1 (F. - 2.)","Check the wiring for short-circuit to 24 V."},
  {SEG_F,SEG_3,"Short-circuit to 0 V at OSSD connection 1 (F. - 3.)","Check the wiring for short-circuit to 0 V."},
  {SEG_F,SEG_4,"Overcurrent on OSSD connection 2 (F. - 4.)","Check the switching element connected (contactor, relay). Replace, if necessary. Check the wiring for short-circuit to 0 V."},
  {SEG_F,SEG_5,"Short-circuit to 24 V at OSSD connection 2 (F. - 5.)","Check the wiring for short-circuit to 24 V."},
  {SEG_F,SEG_6,"Short-circuit to 0 V at OSSD connection 2 (F. - 6.)","Check the wiring for short-circuit to 0 V."},
  {SEG_F,SEG_7,"Short-circuit between OSSD connection 1 and 2 (F. - 7.)","Check the wiring and rectify the error."},
  {SEG_F,SEG_9,"General OSSD wiring error (F. - 9.)","Check the complete wiring of the OSSDs."},
  {SEG_G,SEG_0,"Device is addressed as guest (G.)","No error. The symbol is displayed for approx. 2 seconds on switching on a device that is addressed as a guest."},
  {SEG_H,SEG_0,"Device is addressed as host (H.)","No error. The symbol is displayed for approx. 2 seconds on switching on a device that is addressed as a host."},
  {SEG_1_,SEG__1,"The S3000 is receiving no measured values within a range of at least 90° (measuring range maximum 49 m) it thus is not detecting any obstacles such as e.g. building walls. (1_. - _1.)","For the correct function of the safety laser scanner, always ensure that measured values are received within a range of 90°; this range can be moved as required within the scan range."},
  {SEG_1_,SEG_2,"Device is dazzled (1_. - 2.)","Check whether the S3000 is being dazzled by an external light source, e.g. headlight, infrared light sources, stroboscopic light, sun etc. If necessary, re-mount the device."},
  {SEG_1_,SEG_3,"Temperature error. The operating temperature of the S3000 has exceeded the permissible range. (1_. - 3.)","Check whether the S3000 is operated as per the permissible ambient conditions."},
  {SEG_L,SEG_2,"Invalid configuration of the EDM (L.- 2.)","Verify that the machine-side EDM is connected correctly."},
  {SEG_L,SEG_4,"The addresses of both the host device and the guest device may have been set to guest. (or) A device connected via EFI or the connection to the device is defective or disrupted. (L.- 4.)","Disconnect the connection to the host device (see section 6.1.1 on page 97). Check the connected device and the connection to this device."},
  {SEG_L,SEG_9,"There is a short-circuit between the reset input and another input or output or the reset pulse does not comply with the requirements. (L. - 9.)","Check the wiring for cross-circuits. Or: Check whether the reset pulse complies with the requirements (see Fig. 95 on page 143)."},
  {SEG_n,SEG__1,"Input signal for an undefined monitoring case (n. - _1.)","Check the path of the vehicle. Or: Check the operating process of the monitored machine or system. If necessary, check the configuration of the monitoring cases with the aid of the CDS."},
  {SEG_n,SEG_2,"Incorrect sequence on switching the monitoring cases (n. - 2.)","Check the path of the vehicle. Or: Check the operating process of the monitored machine or system. If necessary, check the configuration of the monitoring cases with the aid of the CDS."},
  {SEG_n,SEG_3,"Incorrect operation of the control inputs (n. - 3.)","Check the operation of the digital control inputs."},
  {SEG_n,SEG_4,"Short-circuit on control inputs A1/2 or incorrect operation of A1/2 via EFI (n. - 4.)","Check the wiring, the digital control inputs or the wiring to the devices connected via EFI."},
  {SEG_n,SEG_5,"Short-circuit on control inputs B1/2 or incorrect operation of B1/2 via EFI (n. - 5.)","Check the wiring, the digital control inputs or the wiring to the devices connected via EFI."},
  {SEG_n,SEG_6,"Short-circuit on control inputs C1/2 or incorrect operation of C1/2 via EFI (n. - 6.)","Check the wiring, the digital control inputs or the wiring to the devices connected via EFI."},
  {SEG_n,SEG_7,"Short-circuit on control inputs D1/2 or incorrect operation of D1/2 via EFI (n. - 7.)","Check the wiring, the digital control inputs or the wiring to the devices connected via EFI."},
  {SEG_n,SEG_8,"Incorrect operation of E1/2 via EFI (n. - 8.)","Check the wiring, the digital control inputs or the wiring to the devices connected via EFI."},
  {SEG_o,SEG_0,"Park/stand-by mode (see section 4.10.7 on page 64); the OSSDs are in the OFF state; the laser is deactivated. (o.)","No error. Readiness is re-established by switching to a different monitoring case or withdrawing the stand-by bit via EFI."},
  {SEG_P,SEG_0,"A device connected via EFI reports a malfunction. (P.)","Carry out a fault diagnosis of the device connected with the S3000."},
  {SEG_u,SEG_u,"Front screen calibration active (u. flashing)","No error,"},
  {SEG_u,SEG__1,"Channel 1 of the contamination measurement contaminated i.e. the front screen is dirty (u. - _1.)","Clean the front screen."},
  {SEG_u,SEG_2,"Channel 2 of the contamination measurement contaminated i.e. the front screen is dirty (u. - 2.)","Clean the front screen."},
  {SEG_u,SEG_3,"Channel 2 of the contamination measurement contaminated i.e. the front screen is dirty (u. - 3.)","Clean the front screen."},
  {SEG_u,SEG_4,"Channel 2 of the contamination measurement contaminated i.e. the front screen is dirty (u. - 4.)","Clean the front screen."},
  {SEG_u,SEG_5,"Channel 2 of the contamination measurement contaminated i.e. the front screen is dirty (u. - 5.)","Clean the front screen."},
  {SEG_u,SEG_6,"Channel 2 of the contamination measurement contaminated i.e. the front screen is dirty (u. - 6.)","Clean the front screen."},
  {SEG_u,SEG_7,"No front screen fitted or dazzling of the contamination measurement (u. - 7.)","Re-fit the new front screen (then perform front screen calibration). If at the time of the error a front screen was fitted: Check whether the S3000 is being dazzled by an external light source, e.g. headlight, infrared light source, stroboscopic light, sun etc."},
  {SEG_u,SEG_8,"Traceability data incorrect or front screen calibration failed (u. - 8.)","Carry out a front screen calibration or replace the S3000, if necessary."},
  {SEG_u,SEG_9,"Traceability data incorrect or front screen calibration failed (u. - 8.)","Carry out a front screen calibration or replace the S3000, if necessary."},
  {SEG_y,SEG__1,"Internal error in the S3000 (y. - _1.)","Replace the S3000."},
  {SEG_y,SEG_2,"Internal I/O module error(y. - 2.)","Replace the I/O module."},
  {SEG_y,SEG_3,"I/O module/sensor head device combination invalid (y. - 3.)","Check whether the correct I/O module has been used, and replace if necessary."},
};

int SickS3000::GetDeviceError(uint8_t first_char, uint8_t second_char, std::string* error, std::string* suggested_action)
{
  if(first_char != 0x00 || second_char != 0x00)
  {
    for(int i = 0; i < 63; i++)
    {
      if(first_char == s3000_errors[i].first_char && second_char == s3000_errors[i].second_char)
      {
        *error = s3000_errors[i].cause;
        *suggested_action = s3000_errors[i].suggested_action;
        return 0;
      }
    }
    *error = "Invalid error code!";
    *suggested_action = "Unable to suggest action.";
    ROS_WARN("Invalid error code!");
    return -1;
  }
  *error = "Currently there is no error.";
  *suggested_action = "Currently there is no error.";
  return -1;
}

int SickS3000::getDiagnosticInfo(bool host, uint16_t* scid, uint8_t* seven_seg_first_char, uint8_t* seven_seg_second_char)
{
  if (serial==NULL)
  {
    return -1;
  }

  uint8_t device_address_ = ((host) ? 0x07 : 0x08);
  uint16_t crc = 0x0000;
  char rx_data[256] = {0xFF};
  int bytes_read = 0;
  int bytes_written = 0;
  int reply_index = 0;
  bool read_error = true;

  char interrupt_byte[1] = {0x41};
  char reserve_token[20] = {0x00,0x00,0x41,0x44,0x19,0x00,0x00,0x05,0xFF,device_address_,0x19,0x00,0x00,0x05,0xFF,device_address_,device_address_,0x0F,0x00,0x00};
  char release_token[20] = {0x00,0x00,0x41,0x44,0x19,0x00,0x00,0x05,0xFF,device_address_,0x19,0x00,0x00,0x05,0xFF,device_address_,0x00,0x00,0x00,0x00};
  char request_scid_block[10] = {0x00,0x00,0x45,0x44,0x17,0x00,0x00,0x0F,0xFF,device_address_};
  char request_block_11[10] = {0x00,0x00,0x45,0x44,0x0B,0x00,0x00,0x56,0xFF,device_address_};

  crc = CreateCRC((uint8_t*)&reserve_token[10], 8);
  reserve_token[18] = (crc & 0x00FF);
  reserve_token[19] = (crc & 0xFF00) >> 8;

  crc = CreateCRC((uint8_t*)&release_token[10], 8);
  release_token[18] = (crc & 0x00FF);
  release_token[19] = (crc & 0xFF00) >> 8;

  //Interrupt continuous mode
  serial->WritePort(interrupt_byte, &bytes_written, 1);
  ros::Duration(0.1).sleep();

  //Reserve token
  serial->WritePort(reserve_token, &bytes_written, 20);
  ros::Duration(0.1).sleep();

  //Read SCID block
  serial->WritePort(request_scid_block, &bytes_written, 10);
  ros::Duration(0.1).sleep();

  serial->BlockOnRead(100);
  bytes_read = serial->ReadPort(rx_data);
  ROS_INFO("Host: %d Reply: %02X %02X %02X %02X %02X %02X %02X %02X", 
    host, rx_data[0], rx_data[1], rx_data[2], rx_data[3],
    rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

  *scid = ((0x00FF & rx_data[35]) << 8) + (0x00FF & rx_data[34]);
  ROS_INFO("SCID: %04X", *scid);
  
  //Read operating data block
  serial->WritePort(request_block_11, &bytes_written, 10);
  ros::Duration(0.1).sleep();

  serial->BlockOnRead(100);
  bytes_read = serial->ReadPort(rx_data);

  //Search for the block 11 read reply
  for(int i = 0; i < bytes_read; i++)
  {
    if(rx_data[i] == 0x0B)
    {
      reply_index = i;
      read_error = false;
      //ROS_INFO("Found 0x0B @ %d!", reply_index);
      break;
    }
  }

  //Release token
  serial->WritePort(release_token, &bytes_written, 20);
  ros::Duration(0.1).sleep();

  serial->BlockOnRead(100);
  serial->ReadPort(rx_data, &bytes_read, 4);
  ROS_INFO("Host: %d Reply: %02X %02X %02X %02X", host, rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

  if(read_error)
  {
    return -1;
  }

  *seven_seg_first_char = static_cast<unsigned char>(rx_data[reply_index+8]);
  *seven_seg_second_char = static_cast<unsigned char>(rx_data[reply_index+9]);

  ROS_INFO("Host: %d device_state: %02X led_code: first %02X second %02X", host,
    (static_cast<unsigned char>(rx_data[reply_index+6]) & 0x03),
    static_cast<unsigned char>(rx_data[reply_index+8]),
    static_cast<unsigned char>(rx_data[reply_index+9]));

  return 0;
}
