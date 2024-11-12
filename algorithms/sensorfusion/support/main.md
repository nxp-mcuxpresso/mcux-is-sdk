Introduction
============

Sensor fusion is a process by which data from several different sensors are fused to compute something more than could be determined by any one sensor alone. An example is computing the orientation of a device in three dimensional space. That orientation is then used to alter the perspective presented by a 3D GUI or game.

The NXP Sensor Fusion Library for Kinetis MCUs (also referred to as Fusion Library or development kit) provides advanced functions for computation of device orientation, linear acceleration, gyro offset and magnetic interference based on the outputs of NXP inertial and magnetic sensors.

Version 7.00 of the sensor fusion development kit has the following features:

* Full source code for the sensor fusion libraries
* IDE-independent software based upon the NXP Kinetis Software Development Kit (KSDK). 
* The Fusion Library no longer requires Processor Expert for component configuration.
* Supports both bare-metal and RTOS-based project development.  Library code is now RTOS agnostic.
* Optional standby mode powers down power hungry sensors when no motion is detected.
* 9-axis Kalman filters require significantly less MIPS to execute
* All option require significantly less memory than those in the Version 5.xx library.
* Full documentation including user manual and fusion data sheet

Included Documentation
======================
* [Sensor Fusion Datasheet] (../NSFK_DS.pdf)
* [Sensor Fusion User Manual] (../NSFK_Prod_UG.pdf)
* [AN5016 - Trigonometry Approximations] (../AN5016 v2p0.pdf)
* [AN5017 - Aerospace Android and Windows Coordinate Systems] (../AN5017 v2p0.pdf)
* [AN5018 - Basic Kalman Filter Theory] (../AN5018 v2p0.pdf)
* [AN5019 - Magnetic Calibration Algorithms] (../AN5019 v2p0.pdf)
* [AN5020 - Determining Matrix Eigenvalues and Eigenvectors by Jacobi Algorithm] (../AN5020 v2p0.pdf)
* [AN5021 - Calculation of Orientation Matrices from Sensor Data] (../AN5021 v2p0.pdf)
* [AN5022 - Quaternion Algebra and Rotations] (../AN5022 v2p0.pdf)
* [AN5023 - Sensor Fusion Kalman Filters] (../AN5023 v2p0.pdf)
* [AN5286 - Precision Accelerometer Calibration] (../AN5286 v1p0.pdf)
