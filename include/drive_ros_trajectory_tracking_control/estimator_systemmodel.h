//
// Created by phillip on 29.05.21.
//

#ifndef ESTIMATOR_SYSTEMMODEL_H
#define ESTIMATOR_SYSTEMMODEL_H

#include <kalman/LinearizedSystemModel.hpp>

namespace Models {

/**
 * @brief System state vector-type for the vehicle
 *
 * This is the System state
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)

    //! Velocity
    static constexpr size_t V = 0;
    //! slip angle
    static constexpr size_t BETA = 1;
    //! yaw angle rate
    static constexpr size_t YAW = 2;
    //! acceleration
    static constexpr size_t ACC = 3;
    //! steering angle front
    static constexpr size_t DELTA_F = 4;
    //! steering angle rear
    static constexpr size_t DELTA_R = 5;

    T v() const { return (*this)[V]; }

    T beta() const { return (*this)[BETA]; }

    T yaw() const { return (*this)[YAW]; }

    T acc() const { return (*this)[ACC]; }

    T delta_f() const { return (*this)[DELTA_F]; }

    T delta_r() const { return (*this)[DELTA_R]; }

    T &v() { return (*this)[V]; }

    T &beta() { return (*this)[BETA]; }

    T &yaw() { return (*this)[YAW]; }

    T &acc() { return (*this)[ACC]; }

    T &delta_f() { return (*this)[DELTA_F]; }

    T &delta_r() { return (*this)[DELTA_R]; }
};

/**
 * @brief System control-input vector-type for the vehicle
 *
 * This is the system control-input
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 3> {
    public:
        KALMAN_VECTOR(Control, T, 3);

        //! acceleration command
        static constexpr size_t ACC_REF = 0;
        //! front steering angle
        static constexpr size_t DELTA_F_REF = 1;
        //! rear steering angle
        static constexpr size_t DELTA_R_REF = 2;

        T acc_ref() const { return (*this)[ACC_REF]; }

        T delta_f_ref() const { return (*this)[DELTA_F_REF]; }

        T delta_r_ref() const { return (*this)[DELTA_R_REF]; }

        T &acc_ref() { return (*this)[ACC_REF]; }

        T &delta_f_ref() { return (*this)[DELTA_F_REF]; }

        T &delta_r_ref() { return (*this)[DELTA_R_REF]; }
    };

/**
 * @brief System model for the vehicle
 *
 * This is the system model
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase> {
    public:
        void setParams(T dt, T *params) {
            this->dt = dt;
            this->m = params[0];
            this->J_z = params[1];
            this->lf = params[2];
            this->lr = params[3];
            this->c = params[4];
            this->T_a = params[5];
            this->T_delta = params[6];
        }

        //! State type shortcut definition
        typedef Models::State<T> S;

        //! Control type shortcut definition
        typedef Models::Control<T> C;


        /**
         * @brief Definition of (non-linear) state transition function
         *
         * This function defines how the system state is propagated through time,
         * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
         * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
         * the system control input \f$u\f$.
         *
         * @param [in] x The system state in current time-step
         * @param [in] u The control vector input
         * @returns The (predicted) system state in the next time-step
         */
        S f(const S &x, const C &u) const {
            //! Predicted state vector after transition
            S x_;
            // tire forces
            double F_lon = m * x.acc() / (cos(x.delta_f()) + cos(x.delta_r()));
            double F_lat_f = c * (x.delta_f() - x.beta() - lf * x.yaw() / x.v());
            double F_lat_r = c * (x.delta_r() - x.beta() + lr * x.yaw() / x.v());

            // system dynamics
            x_.v() = x.v() + (1 / m * (-F_lat_f * sin(x.delta_f() - x.beta()) - F_lat_r *
                    sin(x.delta_r() - x.beta()) + F_lon * cos(x.delta_f() - x.beta()) + F_lon *
                    cos(x.delta_r() - x.beta()))) * dt;
            x_.beta() = x.beta() + (-x.yaw() + 1 / (m * x.v()) * (F_lat_f * cos(x.delta_f() - x.beta()) + F_lat_r *
                    cos(x.delta_r() - x.beta()) + F_lon * sin(x.delta_f() - x.beta()) + F_lon * sin(x.delta_r() -
                    x.beta()))) * dt;
            x_.yaw() = x.yaw() + (1 / J_z * (F_lat_f * lf * cos(x.delta_f() - x.beta()) - F_lat_r * lr *
                    cos(x.delta_r() - x.beta()) + F_lon * lf * sin(x.delta_f() - x.beta()) - F_lon * lr *
                    sin(x.delta_r() - x.beta()))) * dt;
            x_.acc() = x.acc() + (1 / T_a * (u.acc_ref() - x.acc())) * dt;
            x_.delta_f() = x.delta_f() + (1 / T_delta * (u.delta_f_ref() - x.delta_f())) * dt;
            x_.delta_r() = x.delta_r() + (1 / T_delta * (u.delta_r_ref() - x.delta_r())) * dt;

            // Return transitioned state vector
            return x_;
        }

    private:
        // vehicle parameter
        T dt, m, J_z, lf, lr, c, T_a, T_delta;


        // Add here Jacobians for extended Kalman Filter

        /*
    protected:
        /**
         * @brief Update jacobian matrices for the system state transition function using current state
         *
         * This will re-compute the (state-dependent) elements of the jacobian matrices
         * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
         * current state \f$x\f$.
         *
         * @note This is only needed when implementing a LinearizedSystemModel,
         *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
         *       When using a fully non-linear filter such as the UnscentedKalmanFilter
         *       or its square-root form then this is not needed.
         *
         * @param x The current system state around which to linearize
         * @param u The current system control input


        void updateJacobians( const S& x, const C& u )
        {
            // F = df/dx (Jacobian of state transition w.r.t. the state)
            this->F.setZero();

            // partial derivative of x.x() w.r.t. x.x()
            this->F( S::X, S::X ) = ;
        }
        */

    };

} // namespace Models


#endif //ESTIMATOR_SYSTEMMODEL_H
