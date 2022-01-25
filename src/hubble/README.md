**Hubble** is Nova's state estimation system. State estimation is the calculation of the vehcile's
current position, orientation, velocity, and acceleration. With a good state estimate, the jobs of
downstream nodes become much easier!

Hubble's core is a single Unscented Kalman Filter (UKF) provided by the open-source
`robot_localization` package. UKFs can take a number of sensor measurements and fuse them to
produce a single estimate.

Ultimately, our UKF should account for sensors across the vehicle, from the GPS to multiple IMUs
to the steering angle sensor and speedometer. Having multiple data sources means fewer points of
failure.

Hubble also includes packages to support our UKF. The **ICP nudger** uses map data and real-time
Lidar data to correct for small biases in the GPS, producing a slow and accurate result. The
`state_estimate_monitoring` package checks that our UKF's results are reasonable. At the moment,
both the ICP Nudger and the `state_estimate_monitoring` package are skeletons-- while they compile,
they don't do much at the moment. Features will be added to them in future releases.