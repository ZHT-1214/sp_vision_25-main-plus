#pragma once

namespace awakening::sentry_brain {

class DogMode: public ModeBase {
public:
    struct Params {
        double go_home_hp_ratio;
        int home_bullet_num;

        void load(const YAML::Node& config) {
            go_home_hp_ratio = config["go_home_hp_ratio"].as<double>();
            home_bullet_num = config["home_bullet_num"].as<int>();
        }
    } params_;

    DogMode(
        rcl::RclcppNode& rcl_node,
        rcl::TF& rcl_tf,
        const YAML::Node& config,
        std::shared_ptr<SerialDriver> serial
    ):
        ModeBase(rcl_node, rcl_tf, config, serial) {
        params_.load(config);
    }

    struct Action {
        GobalState::Pose pose;
        std::optional<Eigen::Vector2d> goal;
        std::string goal_name;
    };

    void send_current_pose() {
        if (!serial_)
            return;
        SentryRefereeSend send;
        send.cmd_ID = SentryRefereeSend::ID;
        send.set_current_pose = std::to_underlying(sentry_pose);
    }

    template<typename Key>
    Action make_move_action(GobalState::Pose arrived_pose = GobalState::Pose::Defend) {
        go<Key>();
        if (!is_reached<Key>())
            return { GobalState::Pose::Move, current_goal_, Key::name };
        return { arrived_pose, current_goal_, Key::name };
    }

    bool low_hp() const {
        double ratio = double(state_.current_hp) / state_.max_hp;
        return (ratio < params_.go_home_hp_ratio || state_.current_hp < 60);
    }

    Action low_hp_action() {
        wait_until(
            [this]() {
                auto action = make_move_action<home_t>();
                sentry_pose = action.pose;
                if (std::abs(state_.current_hp - state_.max_hp) < 50) {
                    AWAKENING_INFO("hp is enough: {}", state_.current_hp);
                    return true;
                }
                AWAKENING_INFO("waiting for hp to recover: {}", state_.current_hp);
                return false;
            },
            std::chrono::duration<double>(1.0)
        );
        return make_move_action<home_t>();
    }

    bool low_bullet() const {
        return state_.current_bullets_ < params_.home_bullet_num;
    }

    Action low_bullet_action() {
        if (state_.home_allowance_bullets_ > 10) {
            return make_move_action<home_t>();
        } else {
            return make_move_action<ally_fort_t>();
        }
    }

    Action enemy_outpost_action() {
        go<ally_highlands_gain_t>();
        if (is_reached<ally_highlands_gain_t>()) {
            return { GobalState::Pose::Attack, current_goal_, ally_highlands_gain_t::name };
        } else {
            return { GobalState::Pose::Move, current_goal_, ally_highlands_gain_t::name };
        }
    }

    Action rebuild_outpost_action() {
        auto action = make_move_action<ally_outpost_t>();
        return action;
    }

    Action patrol_action() {
        patrol<ally_beijing_tunnel_top_t, enemy_jiansudai_head_t>(10.0);
        if (!is_reached<ally_beijing_tunnel_top_t>() && !is_reached<enemy_jiansudai_head_t>()) {
            return { GobalState::Pose::Move, current_goal_, "Patrol" };
        }
        return { GobalState::Pose::Defend, current_goal_, "Patrol" };
    }

public:
    void tick_callback() override {
        send_current_pose();

        auto& map = RMUC2026Map::instance();
        if (state_.current_game_time_ < 0) {
            AWAKENING_INFO("waiting for game start... current_time: {}", state_.current_game_time_);
            return;
        }

        // 默认姿态
        sentry_pose = GobalState::Pose::Defend;
        if (in_home())
            state_.home_allowance_bullets_ = 0;

        // 优先级决策
        std::optional<Action> action;

        if (low_hp())
            action = low_hp_action();
        if (!action && low_bullet())
            action = low_bullet_action();
        if (!action && state_.enemy_outpost_active_)
            action = enemy_outpost_action();
        if (!action && state_.remain_rebuild_outpost_chance_ > 0)
            action = rebuild_outpost_action();
        if (!action)
            action = patrol_action();

        // 应用 Action
        if (action) {
            sentry_pose = action->pose;
            current_goal_ = action->goal;
            current_goal_name_ = action->goal_name;
        }

        // 目标检测覆盖
        if (target_in_big_yaw_.check())
            sentry_pose = GobalState::Pose::Attack;

        // 极低血量保护
        if (state_.current_hp < 10)
            sentry_pose = GobalState::Pose::Defend;
    }
};

} // namespace awakening::sentry_brain