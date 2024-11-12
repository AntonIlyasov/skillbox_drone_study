#ifndef UAVCONTROLSYSTEM_HPP
#define UAVCONTROLSYSTEM_HPP

#include "typesData.hpp"
#include "math.hpp"
#include "motionPlanner.hpp"
#include <cmath>

class UAVControlSystem
{
	public:
		UAVControlSystem(const ParamsControlSystem *paramsControlSystem, const ParamsSimulator *paramsSimulator,
						 const ParamsQuadrotor *paramsQuadrotor, MotionPlanner* motionPlanner);
		VectorXd_t					calculateMotorVelocity(VectorXd_t stateVector, MatrixXd_t targetPoints, double time);
	private:
		const ParamsSimulator		*paramsSimulator;
		const ParamsQuadrotor		*paramsQuadrotor;
		const ParamsControlSystem	*parContrlSyst;
		
		// Ошибки
		Eigen::Vector3d				angularRateError;
		Eigen::Vector3d				angularPositionError;
		Eigen::Vector3d				positionError;
		Eigen::Vector3d				velocityError;
		Eigen::Vector3d				accelerationError;
	
		Eigen::Vector3d				angularRateErrorPast;
		Eigen::Vector3d				angularPositionErrorPast;
		Eigen::Vector3d				positionErrorPast;
		Eigen::Vector3d				velocityErrorPast;
		Eigen::Vector3d				accelerationErrorPast;

		// Интеграл от ошибки
		Eigen::Vector3d				integralAngularRateError;
		Eigen::Vector3d				integralAngleError;
		Eigen::Vector3d				integralPoseError;
		Eigen::Vector3d				integralVelocityError;
		Eigen::Vector3d				integralAccelerationError;

		// Производная от ошибки
		Eigen::Vector3d				derivativeAngularRateError;
		Eigen::Vector3d				derivativeAngleError;
		Eigen::Vector3d				derivativePoseError;
		Eigen::Vector3d				derivativeVelocityError;
		Eigen::Vector3d				derivativeAccelerationError;

		// Целевые параметры управления
		Eigen::Vector3d				desiredPosition;
		Eigen::Vector3d				desiredVelocity;
		Eigen::Vector3d				desiredAcceleration;
		Eigen::Vector3d				desiredTorque;
		Eigen::Vector3d				desiredAngle;
		Eigen::Vector2d				desiredAngleHorizontal;
		Eigen::Vector3d				desiredAngularRate;
		Eigen::Vector3d				desiredAngularAcceleration;

		VectorXd_t					mixerCommands;
		VectorXd_t					stateVector;
		double						time;
		double						timeTrajectory; // время, за которое БЛА пролетает траекторию
		double						timeStopTrajectory; // время, для остановки движения дрона, после достижения конечной точки
		double						indexPoint; // текущая точка, к которой летит БЛА
		bool						stopTime; // отключает расчёт траектории от времени

		MotionPlanner*				motionPlanner;

		VectorXd_t			mixer();
		void				fillDesiredPosition(MatrixXd_t targetPoints);
		void				PIDThrust();
		void				PIDPosition();
		void				PIDAngles();
		void				PIDAngularRate();
		bool				checkRadius(VectorXd_t targetPoints);
		void				saturation(double &arg, double min, double max);
		double				commandThrustToOmegaRotors(double commandThrust);
};

#endif