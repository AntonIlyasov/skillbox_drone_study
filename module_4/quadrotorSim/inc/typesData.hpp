#ifndef TYPESDATA_HPP
#define TYPESDATA_HPP

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>

const std::string userPath = "..";
const std::string pathQuadModelConfig = userPath + "/config/quadModelConfig.yaml";
const std::string pathQuadControlSystemConfig = userPath + "/config/quadControlSystem.yaml";

// Параметры математической модели
struct ParamsQuadrotor
{
	double			mass; // масса квадрокоптера
	double			lengthOfFlyerArms; // длина лучей
	double			b; // коэффициент тяги
	double			d; // коэффициент сопротивления
	unsigned int	numberOfRotors; // кол-во роторов
	double			Ixx; // тензор инерции
	double			Iyy;
	double			Izz;
	double			maxVelocityRotors;
};

// Параметры симулятора
struct ParamsSimulator
{
	double	dt; // шаг симулятора
	double	realTimeFactor; //фактор реального времени
	double	simulationTotalTime; // общее время симуляции
};

// Параметры системы управления
struct ParamsControlSystem
{
	// Ограничение значений выхода ПИД-регулятора по угловому положению
	double	maxCommandAngle;
	// Ограничения максимального значения выхода ПИД-регулятора по угловой скорости
	double	maxCommandAngularRate;
	// Ограничения максимального значения выхода ПИД-регулятора по угловому ускорению
	double	maxCommandAngularAcceleration;
	// Ограничение по тяге
	double	maxThrust;
	// Коэффициенты ПИД регулятора угловой скорости
	Eigen::Vector3d		KpAngularRate;
	Eigen::Vector3d		KiAngularRate;
	Eigen::Vector3d		KdAngularRate;
	// Коэффициенты ПИД регулятора углового положения
	Eigen::Vector3d		KpAngle;
	Eigen::Vector3d		KiAngle;
	Eigen::Vector3d		KdAngle;
	// Коэффициенты ПИД регулятора пространственного положения
	Eigen::Vector3d		KpPosition;
	Eigen::Vector3d		KiPosition;
	Eigen::Vector3d		KdPosition;
};

// ускорение свободного падения
const double		GRAVITY_ACCELERATION = 9.81;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1>				VectorXd_t;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>	MatrixXd_t;
typedef	std::vector<int>										Vec1i_t;


#endif