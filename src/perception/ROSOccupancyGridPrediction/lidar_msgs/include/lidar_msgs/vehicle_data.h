// #pragma once

#include <vector>
#include <lidar_msgs/vehicle_state.h>

namespace lidar_msgs
{

class VehicleData{

  private:
    lidar_msgs::vehicle_state ros_vehicleState;
    double psi_rad;
    double N_m;
    double E_m;
  public:

    /* Constructors */
    VehicleData();
    VehicleData(const lidar_msgs::vehicle_state& vehicleState);

    /* Setters & Getters */
    void setState(const lidar_msgs::vehicle_state& vehicleState);

    void getPose(double& E_m, double& N_m, double& psi_rad) const;
    void getPoseWrtE(double& E_m, double& N_m, double& psi_rad) const;

    /* Miscellaneous */
};

}
