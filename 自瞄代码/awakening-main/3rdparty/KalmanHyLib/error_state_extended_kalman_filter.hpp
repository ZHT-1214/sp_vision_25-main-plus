#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cassert>
#include <ceres/jet.h>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace kalman_hybird_lib {

template<int N_X, class PredicFunc>
class ErrorStateEKF {
public:
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using Jet = ceres::Jet<double, N_X>;
    using JetMatrixX1 = Eigen::Matrix<Jet, N_X, 1>;

    using UpdateQFunc = std::function<MatrixXX()>;
    using InjectFunc = std::function<void(const MatrixX1&, MatrixX1&)>;
    using BoxMinusFunc = std::function<void(const MatrixX1&, const MatrixX1&, MatrixX1&)>;
    using InjectJetFunc = std::function<void(const JetMatrixX1&, JetMatrixX1&)>;
    using BoxMinusJetFunc =
        std::function<void(const JetMatrixX1&, const JetMatrixX1&, JetMatrixX1&)>;

    ErrorStateEKF() = default;

    template<class Inject, class BoxMinus>
    explicit ErrorStateEKF(
        const PredicFunc& f,
        const UpdateQFunc& u_q,
        const Inject& inject,
        const BoxMinus& box_minus,
        const MatrixXX& P0
    ):
        f(f),
        update_Q(u_q),
        P_delta(P0) {
        set_inject_state(inject);
        set_box_minus_state(box_minus);
    }

    void set_state(const MatrixX1& x0) noexcept {
        x_nominal = x0;
        delta_x.setZero();
    }

    void set_update_Q(const UpdateQFunc& u_q) {
        update_Q = u_q;
    }
    void set_predict_func(const PredicFunc& f_) {
        f = f_;
    }
    template<class Inject>
    void set_inject_state(const Inject& inject) {
        inject_state = inject;
        inject_state_jet = inject;
    }
    template<class BoxMinus>
    void set_box_minus_state(const BoxMinus& box_minus) {
        box_minus_state = box_minus;
        box_minus_state_jet = box_minus;
    }

    MatrixX1 predict() noexcept {
        if (inject_state && box_minus_state) {
            const MatrixX1 x_prev = x_nominal;
            MatrixX1 x_pred;
            f(x_prev.data(), x_pred.data());
            x_nominal = x_pred;

            JetMatrixX1 x_nominal_jet;
            JetMatrixX1 x_prev_jet;
            for (int i = 0; i < N_X; ++i) {
                x_nominal_jet[i] = Jet(x_nominal[i]);
                x_prev_jet[i] = Jet(x_prev[i]);
            }

            JetMatrixX1 delta_jet;
            for (int i = 0; i < N_X; ++i) {
                delta_jet[i] = Jet(0.0);
                delta_jet[i].v[i] = 1.0;
            }

            JetMatrixX1 x_pert_jet = x_prev_jet;
            inject_state_jet(delta_jet, x_pert_jet);

            JetMatrixX1 x_pert_pred_jet;
            f(x_pert_jet.data(), x_pert_pred_jet.data());

            JetMatrixX1 delta_pred_jet;
            box_minus_state_jet(x_nominal_jet, x_pert_pred_jet, delta_pred_jet);
            for (int i = 0; i < N_X; ++i)
                F.row(i) = delta_pred_jet[i].v.transpose();

            delta_x = F * delta_x;

            Q = update_Q();
            P_delta = F * P_delta * F.transpose() + Q;
            P_delta = 0.5 * (P_delta + P_delta.transpose());

            return x_nominal;
        }

        std::array<ceres::Jet<double, N_X>, N_X> x_jet;

        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_nominal[i];
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;
        }

        std::array<ceres::Jet<double, N_X>, N_X> x_pred_jet;
        f(x_jet.data(), x_pred_jet.data());

        for (int i = 0; i < N_X; ++i) {
            x_nominal[i] = x_pred_jet[i].a;
            F.row(i) = x_pred_jet[i].v.transpose();
        }

        // error propagation
        delta_x = F * delta_x;

        Q = update_Q();
        P_delta = F * P_delta * F.transpose() + Q;

        // enforce symmetry
        P_delta = 0.5 * (P_delta + P_delta.transpose());

        return x_nominal;
    }

    template<int N_Z, class MeasureFunc, class UpdateRFunc, class ResidualFunc>
    MatrixX1 update(
        const Eigen::Matrix<double, N_Z, 1>& z,
        const MeasureFunc& h,
        const UpdateRFunc& update_R,
        const ResidualFunc& cal_residual
    ) noexcept {
        using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;
        using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
        using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
        using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;

        MatrixX1 delta_iter = delta_x;
        MatrixXX P_iter = P_delta;

        MatrixZX H;
        MatrixXZ K;
        MatrixZZ R;

        for (int iter = 0; iter < iteration_num; ++iter) {
            MatrixX1 x_eval = x_nominal;

            // inject error-state
            if (inject_state)
                inject_state(delta_iter, x_eval);
            else
                x_eval += delta_iter;

            // auto diff
            std::array<ceres::Jet<double, N_X>, N_X> x_jet;
            for (int i = 0; i < N_X; ++i) {
                x_jet[i].a = x_eval[i];
                x_jet[i].v.setZero();
                x_jet[i].v[i] = 1.0;
            }

            std::array<ceres::Jet<double, N_X>, N_Z> z_jet;
            h(x_jet.data(), z_jet.data());

            MatrixZ1 z_pred;
            for (int i = 0; i < N_Z; ++i) {
                z_pred[i] = z_jet[i].a;
                H.row(i) = z_jet[i].v.transpose();
            }

            const MatrixZ1 residual = cal_residual(z_pred, z);
            R = update_R(z);

            const MatrixZZ S = H * P_iter * H.transpose() + R;
            const auto ldlt = S.ldlt();

            const MatrixXZ PHt = P_iter * H.transpose();
            K = ldlt.solve(PHt.transpose()).transpose();

            delta_iter.noalias() += K * residual;
        }

        if (inject_state)
            inject_state(delta_iter, x_nominal);
        else
            x_nominal += delta_iter;

        delta_x.setZero();

        const MatrixXX I = MatrixXX::Identity();
        P_delta = (I - K * H) * P_iter * (I - K * H).transpose() + K * R * K.transpose();

        P_delta = 0.5 * (P_delta + P_delta.transpose());

        return x_nominal;
    }

    struct ObsBase {
        virtual ~ObsBase() = default;
        virtual int dim() const = 0;

        virtual void predict(const Eigen::VectorXd& x, Eigen::VectorXd& z_pred) const = 0;

        virtual void
        residual_and_R(const Eigen::VectorXd& z_pred, Eigen::VectorXd& residual, Eigen::MatrixXd& R)
            const = 0;

        virtual void evaluate(
            const Eigen::VectorXd& x,
            Eigen::MatrixXd& H,
            Eigen::VectorXd& residual,
            Eigen::MatrixXd& R
        ) const = 0;
    };

    template<int N_Z, class MeasureFunc, class UpdateRFunc, class ResidualFunc>
    struct ObsImpl: public ObsBase {
        using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;

        MatrixZ1 z;
        MeasureFunc h;
        UpdateRFunc update_R;
        ResidualFunc residual_func;

        ObsImpl(
            const MatrixZ1& z_,
            const MeasureFunc& h_,
            const UpdateRFunc& r_,
            const ResidualFunc& res_
        ):
            z(z_),
            h(h_),
            update_R(r_),
            residual_func(res_) {}

        int dim() const override {
            return N_Z;
        }

        void predict(const Eigen::VectorXd& x, Eigen::VectorXd& z_pred) const override {
            assert(x.size() == N_X);

            std::array<double, N_X> x_data;
            for (int i = 0; i < N_X; ++i)
                x_data[i] = x[i];

            std::array<double, N_Z> z_data;
            h(x_data.data(), z_data.data());

            z_pred.resize(N_Z);
            for (int i = 0; i < N_Z; ++i)
                z_pred[i] = z_data[i];
        }

        void
        residual_and_R(const Eigen::VectorXd& z_pred, Eigen::VectorXd& residual, Eigen::MatrixXd& R)
            const override {
            assert(z_pred.size() == N_Z);

            MatrixZ1 z_pred_fixed;
            for (int i = 0; i < N_Z; ++i)
                z_pred_fixed[i] = z_pred[i];

            residual = residual_func(z_pred_fixed, z);
            R = update_R(z);
        }

        void evaluate(
            const Eigen::VectorXd& x,
            Eigen::MatrixXd& H,
            Eigen::VectorXd& residual,
            Eigen::MatrixXd& R
        ) const override {
            assert(x.size() == N_X);

            std::array<ceres::Jet<double, N_X>, N_X> x_jet;
            for (int i = 0; i < N_X; ++i) {
                x_jet[i].a = x[i];
                x_jet[i].v.setZero();
                x_jet[i].v[i] = 1.0;
            }

            std::array<ceres::Jet<double, N_X>, N_Z> z_jet;
            h(x_jet.data(), z_jet.data());

            H.resize(N_Z, N_X);
            residual.resize(N_Z);
            R.resize(N_Z, N_Z);

            MatrixZ1 z_pred;
            for (int i = 0; i < N_Z; ++i) {
                z_pred[i] = z_jet[i].a;
                H.row(i) = z_jet[i].v.transpose();
            }

            residual = residual_func(z_pred, z);
            R = update_R(z);
        }
    };

    template<int N_Z, class MeasureFunc, class UpdateRFunc, class ResidualFunc>
    auto make_obs(
        const Eigen::Matrix<double, N_Z, 1>& z,
        MeasureFunc&& h,
        UpdateRFunc&& r,
        ResidualFunc&& res
    ) {
        using ObsT = ObsImpl<
            N_Z,
            std::decay_t<MeasureFunc>,
            std::decay_t<UpdateRFunc>,
            std::decay_t<ResidualFunc>>;

        return std::make_shared<ObsT>(
            z,
            std::forward<MeasureFunc>(h),
            std::forward<UpdateRFunc>(r),
            std::forward<ResidualFunc>(res)
        );
    }

    MatrixX1 update_multi(const std::vector<std::shared_ptr<ObsBase>>& obs_list) noexcept {
        int total_dim = 0;
        for (const auto& obs: obs_list)
            total_dim += obs->dim();

        Eigen::MatrixXd H(total_dim, N_X);
        Eigen::VectorXd residual(total_dim);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(total_dim, total_dim);

        MatrixX1 delta_iter = delta_x;
        MatrixXX P_iter = P_delta;
        Eigen::MatrixXd K(N_X, total_dim);

        for (int iter = 0; iter < iteration_num; ++iter) {
            MatrixX1 x_eval = x_nominal;

            if (inject_state)
                inject_state(delta_iter, x_eval);
            else
                x_eval += delta_iter;

            int offset = 0;
            for (auto& obs: obs_list) {
                Eigen::MatrixXd Hk, Rk;
                Eigen::VectorXd rk;

                if (inject_state && box_minus_state) {
                    Eigen::VectorXd z_pred;
                    obs->predict(x_eval, z_pred);
                    obs->residual_and_R(z_pred, rk, Rk);

                    const int d = obs->dim();
                    Hk.resize(d, N_X);
                    constexpr double eps = 1e-6;
                    for (int i = 0; i < N_X; ++i) {
                        MatrixX1 delta_plus = delta_iter;
                        MatrixX1 delta_minus = delta_iter;
                        delta_plus[i] += eps;
                        delta_minus[i] -= eps;

                        MatrixX1 x_plus = x_nominal;
                        MatrixX1 x_minus = x_nominal;
                        inject_state(delta_plus, x_plus);
                        inject_state(delta_minus, x_minus);

                        Eigen::VectorXd z_plus, z_minus;
                        obs->predict(x_plus, z_plus);
                        obs->predict(x_minus, z_minus);

                        Eigen::VectorXd dz, unused_residual;
                        Eigen::MatrixXd unused_R;
                        obs->residual_and_R(z_minus, dz, unused_R);
                        obs->residual_and_R(z_plus, unused_residual, unused_R);
                        dz = unused_residual - dz;
                        Hk.col(i) = -dz / (2.0 * eps);
                    }
                } else {
                    obs->evaluate(x_eval, Hk, rk, Rk);
                }

                int d = obs->dim();
                H.block(offset, 0, d, N_X) = Hk;
                residual.segment(offset, d) = rk;
                R.block(offset, offset, d, d) = Rk;

                offset += d;
            }

            const Eigen::MatrixXd S = H * P_iter * H.transpose() + R;
            const auto ldlt = S.ldlt();

            Eigen::MatrixXd PHt = P_iter * H.transpose();
            K = ldlt.solve(PHt.transpose()).transpose();

            delta_iter.noalias() += K * residual;
        }

        if (inject_state)
            inject_state(delta_iter, x_nominal);
        else
            x_nominal += delta_iter;

        delta_x.setZero();

        const MatrixXX I = MatrixXX::Identity();
        P_delta = (I - K * H) * P_iter * (I - K * H).transpose() + K * R * K.transpose();

        P_delta = 0.5 * (P_delta + P_delta.transpose());

        return x_nominal;
    }

    void set_iteration_num(int n) {
        iteration_num = std::max(1, n);
    }

private:
    PredicFunc f;
    UpdateQFunc update_Q;
    InjectFunc inject_state;
    BoxMinusFunc box_minus_state;
    InjectJetFunc inject_state_jet;
    BoxMinusJetFunc box_minus_state_jet;

    MatrixXX F = MatrixXX::Zero();
    MatrixXX Q = MatrixXX::Zero();

    MatrixX1 x_nominal = MatrixX1::Zero();
    MatrixX1 delta_x = MatrixX1::Zero();
    MatrixXX P_delta = MatrixXX::Identity();

    int iteration_num = 1;
};

} // namespace kalman_hybird_lib
