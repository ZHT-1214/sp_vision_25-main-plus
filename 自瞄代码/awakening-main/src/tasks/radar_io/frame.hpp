#pragma once

#include "tasks/radar_io/crc.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <vector>
namespace awakening::radar_io {
enum class RoboID : uint16_t {
    R1 = 1,
    R2 = 2,
    R3 = 3,
    R4 = 4,
    R5 = 5,
    R6 = 6,
    R7 = 7,
    R8 = 8,
    R9 = 9,
    R10 = 10,
    R11 = 11,
    B1 = 101,
    B2 = 102,
    B3 = 103,
    B4 = 104,
    B5 = 105,
    B6 = 106,
    B7 = 107,
    B8 = 108,
    B9 = 109,
    B10 = 110,
    B11 = 111,
    R1OP = 0X101,
    R2OP = 0X102,
    R3OP = 0X103,
    R4OP = 0X104,
    R5OP = 0X105,
    R6OP = 0X106,
    B6OP = 0x16A,
    B1OP = 0x165,
    B2OP = 0X166,
    B3OP = 0X167,
    B4OP = 0X168,
    B5OP = 0X169,
    REFEREE = 0X8080,

};
struct FrameHeader {
    static constexpr uint8_t SOF = 0xA5;
    uint8_t sof = SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};
inline void push_u8(std::vector<uint8_t>& out, uint8_t value) {
    out.push_back(value);
}

inline void push_u16(std::vector<uint8_t>& out, uint16_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

inline void push_u32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}
inline std::vector<uint8_t> pack_frame(uint16_t cmd_id, const std::vector<uint8_t>& data) {
    static uint8_t seq = 0;

    constexpr size_t header_size = 5;
    constexpr size_t cmd_size = 2;
    constexpr size_t crc16_size = 2;
    std::vector<uint8_t> frame;
    frame.reserve(header_size + cmd_size + data.size() + crc16_size);
    frame.push_back(FrameHeader::SOF);
    push_u16(frame, static_cast<uint16_t>(data.size()));
    frame.push_back(seq);
    frame.push_back(0);
    frame[4] = get_crc8(frame.data(), static_cast<uint32_t>(frame.size() - 1));
    push_u16(frame, cmd_id);
    frame.insert(frame.end(), data.begin(), data.end());

    const uint16_t crc16 = get_crc16(frame.data(), static_cast<uint32_t>(frame.size()));
    push_u16(frame, crc16);

    return frame;
}
template<class T>
inline std::vector<uint8_t> pack_frame(const T& data) {
    return pack_frame(T::CMDID, utils::to_vector(data));
}
struct RoboStatus {
    static constexpr uint16_t CMDID = 0x0201;
    uint8_t robot_id = 9;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __attribute__((packed));
struct RobotMarks {
    bool hero;
    bool engineer;
    bool infantry3;
    bool infantry4;
    bool drone;
    bool sentry;
};
struct RadarMark {
    static constexpr uint16_t CMDID = 0x020C;
    RobotMarks enemy;
    RobotMarks ally;
    static RadarMark create(const uint8_t* data, size_t len) {
        RadarMark r;

        if (len < 2)
            return r;

        uint16_t bits = data[0] | (data[1] << 8);

        auto get = [&](int bit) { return bits & (1u << bit); };

        r.enemy.hero = get(0);
        r.enemy.engineer = get(1);
        r.enemy.infantry3 = get(2);
        r.enemy.infantry4 = get(3);
        r.enemy.drone = get(4);
        r.enemy.sentry = get(5);

        r.ally.hero = get(6);
        r.ally.engineer = get(7);
        r.ally.infantry3 = get(8);
        r.ally.infantry4 = get(9);
        r.ally.drone = get(10);
        r.ally.sentry = get(11);

        return r;
    }
} __attribute__((packed));
struct RadarInfo {
    static constexpr uint16_t CMDID = 0x020E;
    uint8_t double_vulnerability_chance = 0;

    bool enemy_is_double_vulnerable = false;

    uint8_t encryption_level = 1;

    bool can_change_key = false;
    static RadarInfo create(const uint8_t* data, size_t len) {
        RadarInfo r;
        if (len < 1)
            return r;

        uint8_t bits = data[0];

        auto get = [&](int bit) { return bits & (1u << bit); };

        auto get_range = [&](int start, int count) {
            return (bits >> start) & ((1u << count) - 1);
        };

        // bit0-1
        r.double_vulnerability_chance = get_range(0, 2);

        // bit2
        r.enemy_is_double_vulnerable = get(2);

        // bit3-4
        r.encryption_level = get_range(3, 2);

        // bit5
        r.can_change_key = get(5);
        static uint8_t last_level = 255;

        if (last_level != r.encryption_level) {
            last_level = r.encryption_level;
            AWAKENING_INFO("Encryption level changed -> {}", r.encryption_level);
        }
        return r;
    }

} __attribute__((packed));
enum class CMDID : uint16_t {
    RoboStatus = RoboStatus::CMDID,
    RadarMark = RadarMark::CMDID,
    RadarInfo = RadarInfo::CMDID,
};
struct RobotInteractionData {
    static constexpr uint16_t CMDID = 0x0301;
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    // uint8_t user_data[];
    std::vector<uint8_t> user_data;
    template<class T>
    static RobotInteractionData create(uint16_t sender_id, uint16_t receiver_id, const T& data) {
        RobotInteractionData d;
        d.data_cmd_id = T::CMDID;
        d.sender_id = sender_id;
        d.receiver_id = receiver_id;
        d.user_data = utils::to_vector(data);
        return d;
    }
};
struct RadarCmd {
    static constexpr uint16_t CMDID = 0X0121;
    uint8_t radar_cmd;
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
} __attribute__((packed));
struct ToSenrty {
    static constexpr uint16_t CMDID = 0x0200;
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
} __attribute__((packed));
struct MapRobotData {
    static constexpr uint16_t CMDID = 0x0305;
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
    uint16_t ally_hero_position_x;
    uint16_t ally_hero_position_y;
    uint16_t ally_engineer_position_x;
    uint16_t ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x;
    uint16_t ally_aerial_position_y;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
} __attribute__((packed));
struct CustomInfo {
    static constexpr uint16_t CMDID = 0x0308;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} __attribute__((packed));
/*===| 雷达无线链路-对方机器人位置 |===*/
struct RF_Pos {
    int16_t Robo_1_X_cm;
    int16_t Robo_1_Y_cm;
    int16_t Robo_2_X_cm;
    int16_t Robo_2_Y_cm;
    int16_t Robo_3_X_cm;
    int16_t Robo_3_Y_cm;
    int16_t Robo_4_X_cm;
    int16_t Robo_4_Y_cm;
    int16_t Robo_6_X_cm;
    int16_t Robo_6_Y_cm;
    int16_t Robo_5_X_cm;
    int16_t Robo_5_Y_cm;
} __attribute__((packed));

/*===| 雷达无线链路-对方机器人血量 |===*/
struct RF_Hp {
    int16_t Robo_1_HP;
    int16_t Robo_2_HP;
    int16_t Robo_3_HP;
    int16_t Robo_4_HP;
    int16_t reserve;
    int16_t Robo_7_HP;
} __attribute__((packed));

/*===| 雷达无线链路-对方机器人允许发弹量 |===*/
struct RF_Bullet {
    int16_t Robo_1_Bullet;
    int16_t Robo_3_Bullet;
    int16_t Robo_4_Bullet;
    int16_t Robo_6_Bullet;
    int16_t Robo_7_Bullet;
} __attribute__((packed));

/*===| 雷达无线链路-对方金币与占领状态 |===*/
struct RF_State {
    int16_t Remain_Coin;
    int16_t Total_Coin;
    int32_t RFID_State;
} __attribute__((packed));

/*===| 雷达无线链路-干扰波秘钥 |===*/
struct RF_key {
    uint8_t Key[6];
} __attribute__((packed));
struct FromWifi {
    static constexpr uint16_t CMDID = 0x06;
    uint8_t cmd_ID;
    RF_Pos RF_Position_Struct; //对方机器人位置
    RF_Hp RF_HP_Struct; //对方机器人血量
    RF_Bullet RF_Bullet_Struct; //对方机器人允许发弹量
    RF_State RF_Coin_RFID_Struct; //对方金币与占领状态
    RF_key RF_Key_Struct; //干扰波秘钥
    uint32_t rf_info_count;
    uint32_t rf_jam_count;
    static std::optional<FromWifi> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(FromWifi) || data[0] != CMDID)
            return std::nullopt;

        FromWifi out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }
} __attribute__((packed));
struct ToWifi {
    static constexpr uint16_t CMDID = 0x06;
    uint8_t cmd_id;
    uint8_t robot_id;
    uint8_t jam_level;
} __attribute__((packed));
} // namespace awakening::radar_io