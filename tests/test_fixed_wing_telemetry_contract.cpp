#include "FixedWingTelemetryContract.hpp"

#include <cassert>

int main() {
    assert(FixedWingTelemetryContract::schemaVersion == "aerocps.fixed-wing.v1");
    assert(FixedWingTelemetryContract::csvHeader ==
           "schema_version,step,time_s,position_x_m,position_y_m,altitude_m,"
           "velocity_x_mps,velocity_y_mps,velocity_z_mps,attitude_w,attitude_x,"
           "attitude_y,attitude_z,roll_rate_radps,pitch_rate_radps,yaw_rate_radps,"
           "airspeed_mps,angle_of_attack_rad,sideslip_rad");
    return 0;
}
