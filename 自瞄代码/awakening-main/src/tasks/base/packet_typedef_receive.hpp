#pragma once

#include "tasks/base/web.hpp"
#include "utils/common/type_common.hpp"
#include "utils/utils.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <utility>
#include <vector>

namespace awakening {

struct ReceiveRobotData {
    static constexpr uint8_t ID = 0x02;

    uint8_t cmd_ID;
    uint32_t time_stamp_pc; //收到的上一包的PC时间戳
    uint32_t time_stamp_receive_micro; //收到的上一包时STM32的时间戳
    uint32_t time_stamp_send_micro; //发送此包时STM32的时间戳

    float yaw, pitch,
        roll; //坐标系定义： +x:前，+y:左，+z：上，+yaw 左，+pitch 下。+roll 右倾，旋转顺序ZYX
    // float q[4]; // wxyz
    float vyaw, vpitch, vroll;
    float v_x, v_y, v_z;

    float bullet_speed;
    uint8_t detect_color; //0 r 1 b
    uint32_t bullet_count; //发出弹+1
    float operator_yaw_offset;
    float operator_pitch_offset;
    // uint8_t mode;

    static std::optional<ReceiveRobotData> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(ReceiveRobotData) || data[0] != ID)
            return std::nullopt;

        ReceiveRobotData out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }

    void update_log(uint32_t delay) {
        Web::write_log("robo", [&](auto& j) {
            j["delay"] = Web::val(delay);
            j["timestamp_pc"] = Web::val(time_stamp_pc);
            j["timestamp_receive_micro"] = Web::val(time_stamp_receive_micro);
            j["timestamp_send_micro"] = Web::val(time_stamp_send_micro);

            // auto _q = Eigen::Quaternionf(q[0], q[1], q[2], q[3]);

            j["yaw"] = Web::val(yaw);
            j["pitch"] = Web::val(pitch);
            j["roll"] = Web::val(roll);

            j["v_x"] = Web::val(v_x);
            j["v_y"] = Web::val(v_y);
            j["v_z"] = Web::val(v_z);

            j["bullet_speed"] = Web::val(bullet_speed);
            j["detect_color"] = (detect_color == 0 ? "Red" : "Blue");
            j["bullet_count"] = Web::val(bullet_count);
            j["operator_yaw_offset"] = Web::val(operator_yaw_offset);
            j["operator_pitch_offset"] = Web::val(operator_pitch_offset);
            // j["mode"] = Web::val(mode);
        });
    }

} __attribute__((packed));

struct SentryJointState {
    static constexpr uint8_t ID = 0x04;

    uint8_t cmd_ID;
    float big_yaw_in_world;
    uint8_t turtle_state;
    static std::optional<SentryJointState> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(SentryJointState) || data[0] != ID)
            return std::nullopt;

        SentryJointState out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }
    void update_log() {
        Web::write_log("joint", [&](auto& j) {
            j["big_yaw_in_world"] = Web::val(big_yaw_in_world);
            j["turtle_state"] = Web::val(turtle_state);
        });
    }
} __attribute__((packed));
struct SentryRefereeReceive {
    static constexpr uint8_t ID = 0x05;
    uint8_t cmd_ID;
    uint8_t robo_id;
    uint16_t current_hp;
    uint16_t max_hp;
    uint16_t allowance_bullets;
    uint16_t fort_allowance_bullets;
    uint8_t current_pose; //1 为进攻姿态，2 为防御姿态，3 为移动姿态
    int32_t game_time; //没开比赛发-1 ，记得把裁判系统包的remain_time转换
    uint16_t ally_outpost_hp;
    uint16_t ally_base_hp;
    uint8_t ally_outpost_occ_state; //0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
    uint8_t ally_fort_occ_state; //0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领

    //雷达多机通信
    uint8_t enemy_outpost_active;
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;
    static std::optional<SentryRefereeReceive> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(SentryRefereeReceive) || data[0] != ID)
            return std::nullopt;

        SentryRefereeReceive out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }
    void update_log() {
        Web::write_log("referee", [&](auto& j) {
            j["robo_id"] = Web::val(robo_id);
            j["current_hp"] = Web::val(current_hp);
            j["max_hp"] = Web::val(max_hp);
            j["allowance_bullets"] = Web::val(allowance_bullets);
            j["fort_allowance_bullets"] = Web::val(fort_allowance_bullets);
            j["current_pose"] = Web::val(current_pose);
            j["game_time"] = Web::val(game_time);
            j["ally_outpost_hp"] = Web::val(ally_outpost_hp);
            j["ally_base_hp"] = Web::val(ally_base_hp);
            j["ally_outpost_occ_state"] = Web::val(ally_outpost_occ_state);
            j["ally_fort_occ_state"] = Web::val(ally_fort_occ_state);
            j["enemy_outpost_active"] = Web::val(enemy_outpost_active);

            j["opponent_hero_position_x"] = Web::val(opponent_hero_position_x);
            j["opponent_hero_position_y"] = Web::val(opponent_hero_position_y);
            j["opponent_engineer_position_x"] = Web::val(opponent_engineer_position_x);
            j["opponent_engineer_position_y"] = Web::val(opponent_engineer_position_y);
            j["opponent_infantry_3_position_x"] = Web::val(opponent_infantry_3_position_x);
            j["opponent_infantry_3_position_y"] = Web::val(opponent_infantry_3_position_y);
            j["opponent_infantry_4_position_x"] = Web::val(opponent_infantry_4_position_x);
            j["opponent_infantry_4_position_y"] = Web::val(opponent_infantry_4_position_y);
            j["opponent_aerial_position_x"] = Web::val(opponent_aerial_position_x);
            j["opponent_aerial_position_y"] = Web::val(opponent_aerial_position_y);
            j["opponent_sentry_position_x"] = Web::val(opponent_sentry_position_x);
            j["opponent_sentry_position_y"] = Web::val(opponent_sentry_position_y);
        });
    }
} __attribute__((packed));
struct HeroMode {
    static constexpr uint8_t ID = 0x07;
    uint8_t mode; //o:auto_aim 1:blind
    static std::optional<HeroMode> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(HeroMode) || data[0] != ID)
            return std::nullopt;

        HeroMode out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }
};
} // namespace awakening