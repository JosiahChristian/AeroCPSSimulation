#include "TelemetryContract.hpp"

#include <cassert>
#include <string>

int main() {
    assert(TelemetryContract::schemaVersion == "aerocps.telemetry.v1");
    const std::string header(TelemetryContract::csvHeader);
    assert(header.rfind("schema_version,step,time_s", 0) == 0);
    assert(header.find("altitude_m") != std::string::npos);
    assert(header.find("velocity_mps") != std::string::npos);
    assert(header.find("target_altitude_m") != std::string::npos);
    assert(header.find("gravity_mps2") != std::string::npos);
    return 0;
}
