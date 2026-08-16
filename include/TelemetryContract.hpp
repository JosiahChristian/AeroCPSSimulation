#pragma once

#include <string_view>

namespace TelemetryContract {
inline constexpr std::string_view schemaVersion = "aerocps.telemetry.v1";
inline constexpr std::string_view csvHeader =
    "schema_version,step,time_s,altitude_m,velocity_mps,target_altitude_m,gravity_mps2";
}
