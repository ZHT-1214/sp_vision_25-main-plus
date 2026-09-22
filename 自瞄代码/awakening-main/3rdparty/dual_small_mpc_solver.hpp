#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <utility>
namespace talos {
class DualSmallMpcSolver {
public:
    struct AxisConfig {
        float q_pos { 9e6f };
        float q_vel { 0.0f };
        float r { 1.0f };
        float max_acc { 50.0f };
        float state_min { -std::numeric_limits<float>::infinity() };
        float state_max { std::numeric_limits<float>::infinity() };
        bool enable_state_bound { false };
    };

    static constexpr int kAxisCount = 2;
    static constexpr int kYawAxis = 0;
    static constexpr int kPitchAxis = 1;
    static constexpr int kMaxHorizon = 201;
    static constexpr int kMaxInputHorizon = kMaxHorizon - 1;
    using Ptr = std::unique_ptr<DualSmallMpcSolver>;

    [[nodiscard]] static Ptr
    create(float dt, int horizon, float rho, const AxisConfig& yaw, const AxisConfig& pitch) {
        if (horizon > kMaxHorizon) {
            throw std::invalid_argument(
                "horizon exceeds fixed-capacity limit of " + std::to_string(kMaxHorizon)
            );
        }

        DualSmallMpcSolver s;
        s.N_ = horizon;
        s.last_state_index_ = horizon - 1;
        s.last_input_index_ = horizon - 2;
        s.rho_ = rho;

        s.a00_ = 1.0f;
        s.a01_ = dt;
        s.a10_ = 0.0f;
        s.a11_ = 1.0f;
        s.b0_ = 0.0f;
        s.b1_ = dt;
        s.f_pos_ = 0.0f;
        s.f_vel_ = 0.0f;

        auto init = [&](int axis, const AxisConfig& cfg) {
            if (!s.init_axis(axis, cfg)) {
                // return std::unexpected("failed to initialize batched MPC axis");
                throw std::invalid_argument("failed to initialize batched MPC axis");
            }
            return true;
        };
        if (!init(kYawAxis, yaw)) {
            return nullptr;
        }
        if (!init(kPitchAxis, pitch)) {
            return nullptr;
        }

        s.reset_workspace();
        return std::make_unique<DualSmallMpcSolver>(s);
    }

    void set_settings(float abs_tol, int max_iter) noexcept {
        abs_tol_ = abs_tol;
        max_iter_ = max_iter;
    }

    void set_x0(int axis, double pos, double vel) noexcept {
        x_pos_(axis, 0) = static_cast<float>(pos);
        x_vel_(axis, 0) = static_cast<float>(vel);
    }

    void set_x0(double yaw_pos, double yaw_vel, double pitch_pos, double pitch_vel) noexcept {
        set_x0(kYawAxis, yaw_pos, yaw_vel);
        set_x0(kPitchAxis, pitch_pos, pitch_vel);
    }

    void set_ref_col(int axis, int k, float pos, float vel) noexcept {
        qref_pos_(axis, k) = -(q_pos_(axis) + rho_) * pos;
        qref_vel_(axis, k) = -(q_vel_(axis) + rho_) * vel;
        if (k == last_state_index_) {
            update_terminal_cache(axis, pos, vel);
        }
    }

    void
    set_ref_col(int k, float yaw_pos, float yaw_vel, float pitch_pos, float pitch_vel) noexcept {
        qref_pos_.col(k) << -(q_pos_(kYawAxis) + rho_) * yaw_pos,
            -(q_pos_(kPitchAxis) + rho_) * pitch_pos;
        qref_vel_.col(k) << -(q_vel_(kYawAxis) + rho_) * yaw_vel,
            -(q_vel_(kPitchAxis) + rho_) * pitch_vel;
        if (k == last_state_index_) {
            update_terminal_cache(kYawAxis, yaw_pos, yaw_vel);
            update_terminal_cache(kPitchAxis, pitch_pos, pitch_vel);
        }
    }

    void set_reference(
        const Eigen::Ref<const Eigen::MatrixXf>& yaw_ref,
        const Eigen::Ref<const Eigen::MatrixXf>& pitch_ref
    ) noexcept {
        qref_pos_.row(kYawAxis).head(N_) =
            (-(q_pos_(kYawAxis) + rho_) * yaw_ref.row(0).array()).matrix();
        qref_vel_.row(kYawAxis).head(N_) =
            (-(q_vel_(kYawAxis) + rho_) * yaw_ref.row(1).array()).matrix();
        qref_pos_.row(kPitchAxis).head(N_) =
            (-(q_pos_(kPitchAxis) + rho_) * pitch_ref.row(0).array()).matrix();
        qref_vel_.row(kPitchAxis).head(N_) =
            (-(q_vel_(kPitchAxis) + rho_) * pitch_ref.row(1).array()).matrix();

        update_terminal_cache(
            kYawAxis,
            yaw_ref(0, last_state_index_),
            yaw_ref(1, last_state_index_)
        );
        update_terminal_cache(
            kPitchAxis,
            pitch_ref(0, last_state_index_),
            pitch_ref(1, last_state_index_)
        );
    }

    bool solve() noexcept {
        converged_ = false;
        converged_axis_.fill(false);
        iters_ = 0;

        for (int iter = 0; iter < max_iter_; ++iter) {
            const LaneMask active = active_mask();
            if (!active.any()) {
                converged_ = true;
                return true;
            }

            backward_terminal(active);
            for (int i = last_input_index_; i >= 0; --i) {
                backward_step(i, active);
            }

            Lane pri_s = Lane::Zero();
            Lane pri_u = Lane::Zero();
            Lane dua_s = Lane::Zero();
            Lane dua_u = Lane::Zero();
            Lane x_pos = x_pos_.col(0);
            Lane x_vel = x_vel_.col(0);

            for (int i = 0; i <= last_input_index_; ++i) {
                forward_step(i, active, x_pos, x_vel, pri_s, pri_u, dua_s, dua_u);
            }
            terminal_step(active, x_pos, x_vel, pri_s, dua_s);

            ++iters_;

            if ((iters_ % check_term_) == 0) {
                const Lane dua_s_scaled = Lane::Constant(rho_) * dua_s;
                const Lane dua_u_scaled = Lane::Constant(rho_) * dua_u;
                for (int axis = 0; axis < kAxisCount; ++axis) {
                    if (converged_axis_[axis]) {
                        continue;
                    }
                    converged_axis_[axis] = pri_s(axis) < abs_tol_ && pri_u(axis) < abs_tol_
                        && dua_s_scaled(axis) < abs_tol_ && dua_u_scaled(axis) < abs_tol_;
                }
                if (converged_axis_[kYawAxis] && converged_axis_[kPitchAxis]) {
                    converged_ = true;
                    return true;
                }
            }
        }
        return false;
    }

    [[nodiscard]] float state(int axis, int dim, int k) const noexcept {
        return dim == 0 ? x_pos_(axis, k) : x_vel_(axis, k);
    }

    [[nodiscard]] float input(int axis, int k) const noexcept {
        return u_(axis, k);
    }
    [[nodiscard]] bool converged() const noexcept {
        return converged_;
    }
    [[nodiscard]] bool converged(int axis) const noexcept {
        return converged_axis_[axis];
    }
    [[nodiscard]] int iterations() const noexcept {
        return iters_;
    }
    [[nodiscard]] int horizon() const noexcept {
        return N_;
    }

private:
    using Lane = Eigen::Array<float, kAxisCount, 1>;
    using LaneMask = Eigen::Array<bool, kAxisCount, 1>;
    using StateMat = Eigen::Array<float, kAxisCount, kMaxHorizon>;
    using InputMat = Eigen::Array<float, kAxisCount, kMaxInputHorizon>;

    float a00_ { 1.0f };
    float a01_ { 0.0f };
    float a10_ { 0.0f };
    float a11_ { 1.0f };
    float b0_ { 0.0f };
    float b1_ { 0.0f };
    float f_pos_ { 0.0f };
    float f_vel_ { 0.0f };
    float rho_ { 1.0f };

    Lane q_pos_ { Lane::Zero() };
    Lane q_vel_ { Lane::Zero() };
    Lane r_val_ { Lane::Zero() };
    Lane k0_ { Lane::Zero() };
    Lane k1_ { Lane::Zero() };
    Lane pinf00_ { Lane::Zero() };
    Lane pinf01_ { Lane::Zero() };
    Lane pinf10_ { Lane::Zero() };
    Lane pinf11_ { Lane::Zero() };
    Lane quu_inv_ { Lane::Zero() };
    Lane am00_ { Lane::Zero() };
    Lane am01_ { Lane::Zero() };
    Lane am10_ { Lane::Zero() };
    Lane am11_ { Lane::Zero() };
    Lane apf_pos_ { Lane::Zero() };
    Lane apf_vel_ { Lane::Zero() };
    Lane bpf_ { Lane::Zero() };
    Lane terminal_ref_pos_ { Lane::Zero() };
    Lane terminal_ref_vel_ { Lane::Zero() };
    Lane u_min_ { Lane::Zero() };
    Lane u_max_ { Lane::Zero() };
    Lane state_min_ { Lane::Constant(-std::numeric_limits<float>::infinity()) };
    Lane state_max_ { Lane::Constant(std::numeric_limits<float>::infinity()) };

    int N_ { 0 };
    int last_state_index_ { 0 };
    int last_input_index_ { 0 };

    StateMat x_pos_ { StateMat::Zero() };
    StateMat x_vel_ { StateMat::Zero() };
    StateMat p_pos_ { StateMat::Zero() };
    StateMat p_vel_ { StateMat::Zero() };
    StateMat v_cost_pos_ { StateMat::Zero() };
    StateMat v_cost_vel_ { StateMat::Zero() };
    StateMat v_pos_ { StateMat::Zero() };
    StateMat v_vel_ { StateMat::Zero() };
    StateMat g_pos_ { StateMat::Zero() };
    StateMat g_vel_ { StateMat::Zero() };
    StateMat qref_pos_ { StateMat::Zero() };
    StateMat qref_vel_ { StateMat::Zero() };

    InputMat u_ { InputMat::Zero() };
    InputMat d_ { InputMat::Zero() };
    InputMat z_cost_ { InputMat::Zero() };
    InputMat z_ { InputMat::Zero() };
    InputMat y_ { InputMat::Zero() };

    float abs_tol_ { 1e-3f };
    int max_iter_ { 10 };
    int check_term_ { 1 };

    bool converged_ { false };
    std::array<bool, kAxisCount> converged_axis_ {};
    int iters_ { 0 };

    [[nodiscard]] static Lane clamp_lane(const Lane& v, const Lane& lo, const Lane& hi) noexcept {
        return v.cwiseMin(hi).cwiseMax(lo);
    }

    [[nodiscard]] LaneMask active_mask() const noexcept {
        LaneMask mask;
        mask << !converged_axis_[kYawAxis], !converged_axis_[kPitchAxis];
        return mask;
    }

    bool init_axis(int axis, const AxisConfig& cfg) noexcept {
        const Eigen::Vector2f Q_aug { cfg.q_pos + rho_, cfg.q_vel + rho_ };
        const float R_aug = cfg.r + rho_;

        using Mat2 = Eigen::Matrix2f;
        using Row2 = Eigen::Matrix<float, 1, 2>;

        const Eigen::Matrix2f A = (Eigen::Matrix2f {} << a00_, a01_, a10_, a11_).finished();
        const Eigen::Matrix<float, 2, 1> B = (Eigen::Matrix<float, 2, 1> {} << b0_, b1_).finished();

        Mat2 P = rho_ * Mat2::Identity();
        Row2 K_prev = Row2::Zero();
        Row2 K = Row2::Zero();
        Mat2 Pinf = Mat2::Zero();

        for (int i = 0; i < 1000; ++i) {
            const float S = R_aug + (B.transpose() * P * B)(0);
            const float S_inv = 1.0f / S;
            K = S_inv * (B.transpose() * P) * A;
            Pinf = Q_aug.asDiagonal().toDenseMatrix() + A.transpose() * P * (A - B * K);
            if ((K - K_prev).cwiseAbs().maxCoeff() < 1e-5f) {
                break;
            }
            K_prev = K;
            P = Pinf;
        }

        k0_(axis) = K(0, 0);
        k1_(axis) = K(0, 1);

        pinf00_(axis) = Pinf(0, 0);
        pinf01_(axis) = Pinf(0, 1);
        pinf10_(axis) = Pinf(1, 0);
        pinf11_(axis) = Pinf(1, 1);

        const float Quu = R_aug + (B.transpose() * Pinf * B)(0);
        quu_inv_(axis) = 1.0f / Quu;

        const Eigen::Matrix2f AmBKt = (A - B * K).transpose();
        am00_(axis) = AmBKt(0, 0);
        am01_(axis) = AmBKt(0, 1);
        am10_(axis) = AmBKt(1, 0);
        am11_(axis) = AmBKt(1, 1);

        const Eigen::Vector2f f = Eigen::Vector2f::Zero();
        const Eigen::Vector2f APf = AmBKt * Pinf * f;
        apf_pos_(axis) = APf(0);
        apf_vel_(axis) = APf(1);
        bpf_(axis) = (B.transpose() * Pinf * f)(0);

        q_pos_(axis) = cfg.q_pos;
        q_vel_(axis) = cfg.q_vel;
        r_val_(axis) = cfg.r;
        u_min_(axis) = -cfg.max_acc;
        u_max_(axis) = cfg.max_acc;
        state_min_(axis) =
            cfg.enable_state_bound ? cfg.state_min : -std::numeric_limits<float>::infinity();
        state_max_(axis) =
            cfg.enable_state_bound ? cfg.state_max : std::numeric_limits<float>::infinity();
        return true;
    }

    void reset_workspace() noexcept {
        x_pos_.setZero();
        x_vel_.setZero();
        p_pos_.setZero();
        p_vel_.setZero();
        v_cost_pos_.setZero();
        v_cost_vel_.setZero();
        v_pos_.setZero();
        v_vel_.setZero();
        g_pos_.setZero();
        g_vel_.setZero();
        qref_pos_.setZero();
        qref_vel_.setZero();
        u_.setZero();
        d_.setZero();
        z_cost_.setZero();
        z_.setZero();
        y_.setZero();
        terminal_ref_pos_.setZero();
        terminal_ref_vel_.setZero();
        converged_axis_.fill(false);
    }

    void update_terminal_cache(int axis, float pos, float vel) noexcept {
        terminal_ref_pos_(axis) = -(pinf00_(axis) * pos + pinf01_(axis) * vel);
        terminal_ref_vel_(axis) = -(pinf10_(axis) * pos + pinf11_(axis) * vel);
    }

    void backward_terminal(const LaneMask& active) noexcept {
        const Lane p_pos = terminal_ref_pos_
            - Lane::Constant(rho_)
                * (v_cost_pos_.col(last_state_index_) - g_pos_.col(last_state_index_));
        const Lane p_vel = terminal_ref_vel_
            - Lane::Constant(rho_)
                * (v_cost_vel_.col(last_state_index_) - g_vel_.col(last_state_index_));
        p_pos_.col(last_state_index_) = active.select(p_pos, p_pos_.col(last_state_index_));
        p_vel_.col(last_state_index_) = active.select(p_vel, p_vel_.col(last_state_index_));
    }

    void backward_step(int i, const LaneMask& active) noexcept {
        const Lane r = Lane::Constant(-rho_) * (z_cost_.col(i) - y_.col(i));
        const Lane q_pos =
            qref_pos_.col(i) - Lane::Constant(rho_) * (v_cost_pos_.col(i) - g_pos_.col(i));
        const Lane q_vel =
            qref_vel_.col(i) - Lane::Constant(rho_) * (v_cost_vel_.col(i) - g_vel_.col(i));
        const Lane p1_pos = p_pos_.col(i + 1);
        const Lane p1_vel = p_vel_.col(i + 1);

        const Lane d =
            quu_inv_ * (Lane::Constant(b0_) * p1_pos + Lane::Constant(b1_) * p1_vel + r + bpf_);
        const Lane p_pos = q_pos + am00_ * p1_pos + am01_ * p1_vel - k0_ * r + apf_pos_;
        const Lane p_vel = q_vel + am10_ * p1_pos + am11_ * p1_vel - k1_ * r + apf_vel_;

        d_.col(i) = active.select(d, d_.col(i));
        p_pos_.col(i) = active.select(p_pos, p_pos_.col(i));
        p_vel_.col(i) = active.select(p_vel, p_vel_.col(i));
    }

    void forward_step(
        int i,
        const LaneMask& active,
        Lane& x_pos,
        Lane& x_vel,
        Lane& pri_s,
        Lane& pri_u,
        Lane& dua_s,
        Lane& dua_u
    ) noexcept {
        const Lane u = -(k0_ * x_pos + k1_ * x_vel) - d_.col(i);
        u_.col(i) = active.select(u, u_.col(i));

        const Lane z_prev = z_.col(i);
        const Lane z_new = clamp_lane(u + y_.col(i), u_min_, u_max_);
        z_cost_.col(i) = active.select(z_new, z_cost_.col(i));
        z_.col(i) = active.select(z_new, z_.col(i));
        y_.col(i) = active.select(y_.col(i) + (u - z_new), y_.col(i));

        const Lane abs_u_res = (u - z_new).abs();
        const Lane abs_z_res = (z_prev - z_new).abs();
        pri_u = pri_u.max(active.select(abs_u_res, Lane::Zero()));
        dua_u = dua_u.max(active.select(abs_z_res, Lane::Zero()));

        const Lane v_prev_pos = v_pos_.col(i);
        const Lane v_prev_vel = v_vel_.col(i);
        const Lane v_new_pos = clamp_lane(x_pos + g_pos_.col(i), state_min_, state_max_);
        const Lane v_new_vel = x_vel + g_vel_.col(i);

        v_cost_pos_.col(i) = active.select(v_new_pos, v_cost_pos_.col(i));
        v_cost_vel_.col(i) = active.select(v_new_vel, v_cost_vel_.col(i));
        v_pos_.col(i) = active.select(v_new_pos, v_pos_.col(i));
        v_vel_.col(i) = active.select(v_new_vel, v_vel_.col(i));

        const Lane x_minus_v_pos = active.select(x_pos - v_new_pos, Lane::Zero());
        const Lane x_minus_v_vel = active.select(x_vel - v_new_vel, Lane::Zero());

        g_pos_.col(i) = active.select(g_pos_.col(i) + x_minus_v_pos, g_pos_.col(i));
        g_vel_.col(i) = active.select(g_vel_.col(i) + x_minus_v_vel, g_vel_.col(i));

        pri_s = pri_s.max(x_minus_v_pos.abs().max(x_minus_v_vel.abs()));
        dua_s = dua_s.max(active.select(
            (v_prev_pos - v_new_pos).abs().max((v_prev_vel - v_new_vel).abs()),
            Lane::Zero()
        ));

        const Lane next_pos = Lane::Constant(a00_) * x_pos + Lane::Constant(a01_) * x_vel
            + Lane::Constant(b0_) * u + Lane::Constant(f_pos_);
        const Lane next_vel = Lane::Constant(a10_) * x_pos + Lane::Constant(a11_) * x_vel
            + Lane::Constant(b1_) * u + Lane::Constant(f_vel_);
        x_pos_.col(i + 1) = active.select(next_pos, x_pos_.col(i + 1));
        x_vel_.col(i + 1) = active.select(next_vel, x_vel_.col(i + 1));
        x_pos = active.select(next_pos, x_pos_.col(i + 1));
        x_vel = active.select(next_vel, x_vel_.col(i + 1));
    }

    void terminal_step(
        const LaneMask& active,
        const Lane& x_pos,
        const Lane& x_vel,
        Lane& pri_s,
        Lane& dua_s
    ) noexcept {
        const Lane v_prev_pos = v_pos_.col(last_state_index_);
        const Lane v_prev_vel = v_vel_.col(last_state_index_);
        const Lane v_new_pos =
            clamp_lane(x_pos + g_pos_.col(last_state_index_), state_min_, state_max_);
        const Lane v_new_vel = x_vel + g_vel_.col(last_state_index_);

        v_cost_pos_.col(last_state_index_) =
            active.select(v_new_pos, v_cost_pos_.col(last_state_index_));
        v_cost_vel_.col(last_state_index_) =
            active.select(v_new_vel, v_cost_vel_.col(last_state_index_));
        v_pos_.col(last_state_index_) = active.select(v_new_pos, v_pos_.col(last_state_index_));
        v_vel_.col(last_state_index_) = active.select(v_new_vel, v_vel_.col(last_state_index_));

        const Lane x_minus_v_pos = active.select(x_pos - v_new_pos, Lane::Zero());
        const Lane x_minus_v_vel = active.select(x_vel - v_new_vel, Lane::Zero());

        g_pos_.col(last_state_index_) = active.select(
            g_pos_.col(last_state_index_) + x_minus_v_pos,
            g_pos_.col(last_state_index_)
        );
        g_vel_.col(last_state_index_) = active.select(
            g_vel_.col(last_state_index_) + x_minus_v_vel,
            g_vel_.col(last_state_index_)
        );

        pri_s = pri_s.max(x_minus_v_pos.abs().max(x_minus_v_vel.abs()));
        dua_s = dua_s.max(active.select(
            (v_prev_pos - v_new_pos).abs().max((v_prev_vel - v_new_vel).abs()),
            Lane::Zero()
        ));
    }
};
} // namespace talos
