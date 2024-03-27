//
// Simple example to demonstrate how to imitate a smart battery.
//
// Author: Julian Oes <julian@oes.ch>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;

static void subscribe_armed(Telemetry& telemetry);
static void send_battery_status(MavlinkPassthrough& mavlink_passthrough);
static void request_position_stream(MavlinkPassthrough& mavlink_passthrough);
void subscribe_global_position(Telemetry& telemetry);
void subscribe_velocity_body(Telemetry& telemetry);

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk = Mavsdk();
    auto mav_config = Mavsdk::Configuration(Mavsdk::Configuration::UsageType::GroundStation);
    mavsdk.set_configuration(mav_config);

    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting to discover system...\n";
    while (mavsdk.systems().size() == 0) {
        std::cerr << "No system found, waiting...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Instantiate first found system and plugins.
    auto system = mavsdk.systems().at(0);
    auto telemetry = Telemetry{system};
    auto mavlink_passthrough = MavlinkPassthrough{system};
 
    //Subscribe to armed status messages.
    //You can test this on SITL using MAVProxy by typing "arm throttle" and "disarm throttle".
    //subscribe_armed(telemetry);
    subscribe_global_position(telemetry);
    //subscribe_velocity_body(telemetry);

    // Request odometry messages.
    //request_position_stream(mavlink_passthrough);

    while (true) {
        send_battery_status(mavlink_passthrough);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

void subscribe_armed(Telemetry& telemetry)
{
    telemetry.subscribe_armed(
        [](bool is_armed) { std::cout << (is_armed ? "armed" : "disarmed") << '\n'; });
}

void subscribe_global_position(Telemetry& telemetry) {
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Position: " << position.latitude_deg << ", " << position.longitude_deg << ", "
                  << position.absolute_altitude_m << '\n';
    });

    telemetry.subscribe_position_velocity_ned([](Telemetry::PositionVelocityNed position_velocity_ned) {
        std::cout << "Velocity: " << position_velocity_ned.velocity.north_m_s << ", " << position_velocity_ned.velocity.east_m_s << ", "
                  << position_velocity_ned.velocity.down_m_s << '\n';
    });
}



void subscribe_velocity_body(Telemetry& telemetry) {
    telemetry.subscribe_attitude_angular_velocity_body([](Telemetry::AngularVelocityBody angular_velocity_body) {
        std::cout << "Angular velocity: " << angular_velocity_body.roll_rad_s << ", " << angular_velocity_body.pitch_rad_s << ", "
                  << angular_velocity_body.yaw_rad_s << '\n';
    });

}


void request_position_stream(MavlinkPassthrough& mavlink_passthrough) {

    std::cout << "Requesting position stream...\n";

    mavlink_message_t message;
    //REQUEST DATA STREAM FOR ATTITUDE MSG
    mavlink_msg_request_data_stream_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &message,
        1, // target_system
        0, // target_component
        MAV_DATA_STREAM_POSITION, // req_stream_id
        1, // req_message_rate
        1); // start_stop 

    auto result = mavlink_passthrough.send_message(message);

    if (result != MavlinkPassthrough::Result::Success) {
        std::cerr << "Failed to send message: " << result << '\n';
    } else {
        std::cout << "Requested position stream successfully\n";
    
    }
}


void send_battery_status(MavlinkPassthrough& mavlink_passthrough)
{
    const uint16_t voltages[10]{
        3700,
        3600,
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max(),
        std::numeric_limits<uint16_t>::max()}; // mV

    const uint16_t voltages_ext[4]{0, 0, 0, 0};

    mavlink_message_t message;
    mavlink_msg_battery_status_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &message,
        0, // id
        MAV_BATTERY_FUNCTION_ALL, // battery_function
        MAV_BATTERY_TYPE_LION, // type
        2500, // 100*temperature C
        &voltages[0],
        4000, // 100*current_battery A
        1000, // current_consumed, mAh
        -1, // energy consumed hJ
        80, // battery_remaining %
        3600, // time_remaining
        MAV_BATTERY_CHARGE_STATE_OK,
        voltages_ext,
        MAV_BATTERY_MODE_UNKNOWN, // mode
        0); // fault_bitmask

    mavlink_passthrough.send_message(message);
}
