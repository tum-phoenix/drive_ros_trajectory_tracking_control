//
// Created by phillip on 03.06.21.
//

#ifndef ESTIMATOR_MEASUREMENTMODELS_H
#define ESTIMATOR_MEASUREMENTMODELS_H

#include <kalman/LinearizedMeasurementModel.hpp>
#include <drive_ros_trajectory_tracking_control/estimator_systemmodel.h>

namespace Models{
/**
 * @brief Measurement vector measuring vehicles velocity and steering angles
 *
 * @param T Numeric scalar type
 */
    template<typename T>
    class VehicleStateMeasurement : public Kalman::Vector<T, 3> {
    public:
        KALMAN_VECTOR(VehicleStateMeasurement, T, 3)

        //! velocity measurement from wheel encoders
        static constexpr size_t V_MEAS = 0;

        //! front steering angle measurement
        static constexpr size_t DELTA_F_MEAS = 1;

        //! rear steering angle measurement
        static constexpr size_t DELTA_R_MEAS = 2;

        T v_meas() const { return (*this)[V_MEAS]; }

        T delta_f_meas() const { return (*this)[DELTA_F_MEAS]; }

        T delta_r_meas() const { return (*this)[DELTA_R_MEAS]; }

        T &v_meas() { return (*this)[V_MEAS]; }

        T &delta_f_meas() { return (*this)[DELTA_F_MEAS]; }

        T &delta_r_meas() { return (*this)[DELTA_R_MEAS]; }
    };

/**
 * @brief Measurement model for measuring the velocity and steering angles
 *
 * This is the measurement model for measuring the velocity and the steering angles of the vehicle
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
    template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
    class VehicleStateMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VehicleStateMeasurement<T>, CovarianceBase> {
    public:
    //! State type shortcut definition
    typedef Models::State<T> S;

    //! Measurement type shortcut definition
    typedef Models::VehicleStateMeasurement<T> M;


    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S &x) const {
        M measurement;

        // velocity
        measurement.v_meas() = x.v();

        // front steering angle
        measurement.delta_f_meas() = x.delta_f();

        // rear steering angle
        measurement.delta_r_meas() = x.delta_r();

        return measurement;
    }
    // for Extended Kalman Filter add here Jacobians:
    /*
protected:

     **
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
    void updateJacobians( const S& x )
    {
        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();

        // partial derivative of meas.v_meas() w.r.t. x.v()
        this->H( M::V_MEAS, S::V ) =;
    }
     */
};


/**
* @brief Measurement vector measuring vehicles acceleration and yaw rate
*
* @param T Numeric scalar type
*/
template<typename T>
class ImuStateMeasurement : public Kalman::Vector<T, 2> {
public:
    KALMAN_VECTOR(ImuStateMeasurement, T, 2)

    //! acceleration measurement
    static constexpr size_t ACC_MEAS = 0;

    //! yaw rate measurement
    static constexpr size_t YAW_MEAS = 1;

    T acc_meas() const { return (*this)[ACC_MEAS]; }

    T yaw_meas() const { return (*this)[YAW_MEAS]; }

    T &acc_meas() { return (*this)[ACC_MEAS]; }

    T &yaw_meas() { return (*this)[YAW_MEAS]; }
};

/**
 * @brief Measurement model for measuring the acceleration and yaw rate
 *
 * This is the measurement model for measuring the acceleration and the yaw rate of the vehicle
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class ImuStateMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, ImuStateMeasurement<T>, CovarianceBase>
{
public:
//! State type shortcut definition
typedef Models::State<T> S;

//! Measurement type shortcut definition
typedef Models::ImuStateMeasurement<T> M;


/**
* @brief Definition of (possibly non-linear) measurement function
 *
* This function maps the system state to the measurement that is expected
* to be received from the sensor assuming the system is currently in the
* estimated state.
*
* @param [in] x The system state in current time-step
* @returns The (predicted) sensor measurement for the system state
*/
M h(const S &x) const {
    M measurement;

    // acceleration
    measurement.acc_meas() = x.acc();

    // yaw rate
    measurement.yaw_meas() = x.yaw();

    return measurement;
}
// for Extended Kalman Filter add here Jacobians:
/*
protected:

/**
* @brief Update jacobian matrices for the system state transition function using current state
*
* This will re-compute the (state-dependent) elements of the jacobian matrices
* to linearize the non-linear measurement function \f$h(x)\f$ around the
* current state \f$x\f$.
*
* @note This is only needed when implementing a LinearizedSystemModel,
*       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
*       When using a fully non-linear filter such as the UnscentedKalmanFilter
*       or its square-root form then this is not needed.
*
* @param x The current system state around which to linearize
* @param u The current system control input
void updateJacobians( const S& x )
{
    // H = dh/dx (Jacobian of measurement function w.r.t. the state)
    this->H.setZero();

    // partial derivative of meas.acc_meas() w.r.t. x.acc()
    this->H( M::ACC_MEAS, S::ACC ) =;
}
*/

};

}//namespace Models
#endif //ESTIMATOR_MEASUREMENTMODELS_H
