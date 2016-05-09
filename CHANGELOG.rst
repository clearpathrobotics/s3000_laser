^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package s3000_laser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2016-05-09)
------------------
* Parameterize frequency warning tolerance, change default from 5% to 20%.
* Contributors: Mike Purvis

0.1.4 (2016-02-16)
------------------
* Set range max from param, better default value.
* Contributors: Enrique Fernandez

0.1.3 (2016-02-03)
------------------
* Reduce log outputs from error states.
* Contributors: Mike Purvis

0.1.2 (2015-12-22)
------------------
* Connection/reconnection fix.
  This catches another serial failure mode, this time where select()
  reports the device as ready to read, but no actual data is returned.
  This previously manifested as simply being stuck in an unproductive
  read loop, but now it correctly recovers from this state.
* Contributors: Mike Purvis

0.1.1 (2015-12-18)
------------------
* Eliminate fixed rate loop, block on data receipt
* Rename connection diagnostic, remove self_test.
* Remove getData, use BlockOnRead.
* Switch to DiagnosedPublisher, scoped_ptr.
* Remove tf broadcaster, doesn't belong here.
* Package format 2, CMakeLists cleanup, maintainer change.
* Contributors: Mike Purvis

0.1.0 (2015-09-21)
------------------
* Merge pull request `#2 <https://github.com/clearpathrobotics/s3000_laser/issues/2>`_ from clearpathrobotics/igrrs-441-fix
  Reverted retrieval of params and fixed launch file
* Updated launch file
* Reverted retrieval of params and fixed launch file
  Fixed port param
* Merge pull request `#1 <https://github.com/clearpathrobotics/s3000_laser/issues/1>`_ from clearpathrobotics/igrrs-441
  IGRRS-441: Fix LIDAR diagnostics to return sane results
* Fixed LIDAR status and frequency diagnostics
  Added check for no data from laser
  Updated diagnostics
  Minor refactoring
  Fixed launch file param and minor formatting
  Formatting + removed whitespace
  Updated diagnostic status to use enums
  Formatting
* Added serial device library and launch file to install targets
* Added proper install target in CMakeLists.txt
* remove ~ files
* Changed back to s3000_laser
* First Commit of Catkinized S3000 driver for ROS
* Initial commit
* Contributors: Jonathan Jekir, Mustafa Safri, Prasenjit Mukherjee, abencz, ibaranov, ibaranov-cp, rocksteady_robot
