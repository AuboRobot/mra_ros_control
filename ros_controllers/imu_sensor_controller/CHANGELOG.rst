^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_sensor_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------

0.7.0 (2014-03-28)
------------------

0.6.0 (2014-02-05)
------------------
* Install default config files
* Contributors: Paul Mathieu

0.5.4 (2013-09-30)
------------------
* Added install rules for plugin.xml
* Fix license header string for some files.

0.5.3 (2013-09-04)
------------------
* Removed manifest.xml from all packages to prevent rosdep heirarchy issues in Groovy and Hydro
* Added ignored manifest.xml files, added rule to .gitignore

0.5.2 (2013-08-06)
------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Make hybrid rosbuild-catkin packages.
  Affects force-torque and imu sensor controllers.
* Add package.xml scripts.
* Fix author name typo.
* Fix PLUGINLIB_DECLARE_CLASS depreacated errors.
* Propagate sensor interfaces API changes.
* Fix package URLs.
* Propagate changes in hardware_interface.
  - force-torque and IMU sensors no longer depend on Eigen.
  - The controllers that publish sensor state don't need the Eigen wrappers
  and now use the raw data directly.
* Controller publishing the state of a IMU sensor.

0.4.0 (2013-06-26)
------------------
