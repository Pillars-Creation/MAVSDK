#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h> //key
#include <thread>
#include <future>
#include <chrono>
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/action/action.h"
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "mavsdk/plugins/offboard/offboard.h"

using namespace mavsdk;
using namespace std::this_thread;  // for sleep_for
using namespace std::chrono;      // for seconds

/*termios读取终端键盘输入*/
#define KEY_ESC 27
#define KEY_BRACKET_LEFT 91
#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_RIGHT 67
#define KEY_LEFT 68
static struct termios original_termios = {0};
void reset_input_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
}
void set_input_mode() {
    struct termios tattr;
    tcgetattr(STDIN_FILENO, &tattr);
    original_termios = tattr; // 保存原始终端设置

    atexit(reset_input_mode); // 注册程序退出时调用reset_input_mode函数

    // 关闭缓冲，通过read直接获取键盘输入字符
    tattr.c_lflag &= ~(ICANON | ECHO);
    tattr.c_cc[VMIN] = 1;  // Minimum number of characters to read
    tattr.c_cc[VTIME] = 0; // Timeout (in deciseconds)

    tcsetattr(STDIN_FILENO, TCSANOW, &tattr);
}



/*offboard模式控制飞行*/
bool setup_offboard(mavsdk::Offboard& offboard)
{
    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityNedYaw initial_velocity = {0};
    printf("Setting initial setpoint...\n");
    offboard.set_velocity_ned(initial_velocity);//提供一个初始设置点

    printf("Starting Offboard mode...\n");
    const Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Starting offboard failed: " << offboard_result << '\n';
        return false;
    } else {
        printf("Starting offboard successed\n");
    }

    offboard.set_velocity_ned(initial_velocity);//防止通信延迟或丢包导致的控制指令无法及时生效，再次发送，确保模式切换后有一个有效的速度设置点。
    sleep_for(seconds(1)); // Ensure initial setpoint is sent
    return true;
}

bool stop_offboard(mavsdk::Offboard& offboard)
{
    Offboard::Result offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    } else {
        printf("Offboard stop successed\n");
    }
    sleep_for(seconds(1));
    return true;
}

bool offboard_ctrl_ned(mavsdk::Offboard& offboard, float north_m_s, float east_m_s, float down_m_s, float yaw_deg, uint32_t time_ms)
{
    Offboard::VelocityNedYaw set_point = {0};
    set_point.north_m_s = north_m_s;
    set_point.east_m_s = east_m_s;
    set_point.down_m_s = down_m_s;
    set_point.yaw_deg = yaw_deg;
    
    offboard.set_velocity_ned(set_point);
    sleep_for(milliseconds(time_ms));
    return true;
}



int main(int argc, char** argv)
{
    /*创建与无人机的通信*/
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};

    const std::string url = "udp://0.0.0.0:14540";
    ConnectionResult connection_result = mavsdk.add_any_connection(url);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    //寻找在线的无人机
    printf("Waiting to discover system ...\n");
    std::optional<std::shared_ptr<mavsdk::System>> system;
    while (!system) {
        system = mavsdk.first_autopilot(3.0);
    }
    printf("discover a autopilot system ...\n");
    
    // Instantiate plugins.
    Telemetry telemetry = Telemetry{system.value()};
    Action action = Action{system.value()};
    Offboard offboard = Offboard{system.value()};
    
#if 0
    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        printf("Setting rate failed: %d\n", set_rate_result);
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
        printf("Altitude: %.2f m\n", position.relative_altitude_m);
    });
#endif
    printf("Waiting for system to be ready\n");
    // while (telemetry.health_all_ok() != true) {
    //     sleep_for(seconds(1));
    // }
    printf("System is ready\n");

    /*解锁与起飞*/
    printf("Arming ...\n");
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    
    printf("Taking off ...\n");
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    std::promise<void> in_air_promise;
    std::future<void> in_air_future = in_air_promise.get_future();  
    Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
        [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                printf("Taking off has finished\n");
                telemetry.unsubscribe_landed_state(handle);
                in_air_promise.set_value();
            }
        });
    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        printf("Takeoff timed out.\n");
        return 1;
    }
    
    if (false == setup_offboard(offboard)) {
        printf("setup_offboard failed\n");
        return 1;
    }

    const float side_length = 2.0;
    const uint32_t flying_timems = 10;
    
    /*读取键盘方向键输入，执行飞行操作*/
    printf("控制无人机飞行（输入键盘方向键）、退出（其它任意键键）\n");
    set_input_mode();
    char c = 0;
    while (1) {
        offboard_ctrl_ned(offboard, 0, 0, 0, 0, 0);
        read(STDIN_FILENO, &c, 1);
        if (c == KEY_ESC) {
            read(STDIN_FILENO, &c, 1);
            if (c == KEY_BRACKET_LEFT) {
                read(STDIN_FILENO, &c, 1);
                switch (c) {
                    case KEY_UP: {
                        printf("Flying north...\n");
                        offboard_ctrl_ned(offboard, side_length, 0, 0, 0, flying_timems);
                    } break;
                    
                    case KEY_DOWN: {
                        printf("Flying south...\n");
                        offboard_ctrl_ned(offboard, -side_length, 0, 0, 180.0, flying_timems);
                    } break;
                    
                    case KEY_RIGHT: {
                        printf("Flying east...\n");
                        offboard_ctrl_ned(offboard, 0, side_length, 0, 90.0, flying_timems);
                    } break;
                    
                    case KEY_LEFT: {
                        printf("Flying west...\n");
                        offboard_ctrl_ned(offboard, 0, -side_length, 0, 270.0, flying_timems);
                    } break;
                    
                    default: {
                        // 处理其他的ESC序列或无效输入
                        printf("-----------------Unknown key: %d\n", c);
                    } break;
                }
            }
        } else {
            // 处理其他字符输入
            printf("-----------------Other key pressed: %c, then exit\n", c);
            break;
        }
    }

    if (false == stop_offboard(offboard)) {
        printf("stop_offboard failed\n");
        return 1;
    }

    /*退出操作，飞机着陆，上锁*/
    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));
    printf("Landing ...\n");
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << '\n';
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        printf("Vehicle is landing ...\n");
        sleep_for(seconds(1));
    }
    printf("Landed!\n");

#if 1
    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
#else
    Action::Result disarm_result = action->disarm();
    if (disarm_result != Action::Result::Success) {
        std::cerr << "Disarming failed: " << disarm_result << '\n';
        return 1;
    } else {
        printf("Disarmed successfully!\n");
    }
#endif
    
    printf("Finished ...\n");

    return 0;
}
