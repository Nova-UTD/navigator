## Pseudocode

1. Receive new/updated trajectory, which is a sequence of trajectory points.
2. Receive NDT pose (position, orientation)
3. v_0 is the pose in 2D space (normalized vector)
4. v_1 is the vector from current location to Traj. Pt. A (normalized)
5. v_2 is the vector from Traj. Pt. A to Traj. Pt. B.
6. Assign constants A, B, C to v_1, v_2, v_3 respectively. Multiply each vector by this constant magnitude.
7. Add the dilated vectors together, e.g. (v_1*A)+(v_2*B)+(v_3*C) and call the resultant vector v_result.
8. Let dp equal the dot product of v_result and v_0 (both are *normalized*).
9. Let angle Theta be the arcos of dp.
10. Construct a VehicleCommandMessage with theta.
11. Publish this message to `/vehicle/vehicle_commands`.