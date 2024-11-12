#include "configLoader.hpp"

/**
 * @brief Чтение файла конфигурации модели системы,
 * 		установка параметров модели согласно конфигурации
 * @param filename - путь к файлу конфигурации
 * @param quadParam - структура параметров модели 
 * @param simParam - структура параметров симулятора
 */
void	loadModelConfig(const std::string &fileName, ParamsQuadrotor &quadParam, ParamsSimulator &simParam)
{
	YAML::Node fs = YAML::LoadFile(fileName);
	// Устанавливаем параметры математической модели
	quadParam.mass 					= fs["mass"].as<double>();
	quadParam.lengthOfFlyerArms 	= fs["lengthOfFlyerArms"].as<double>();
	quadParam.b 					= fs["motorThrustCoef"].as<double>();
	quadParam.d 					= fs["motorResistCoef"].as<double>();
	quadParam.numberOfRotors 		= fs["numberOfRotors"].as<unsigned int>();
	quadParam.Ixx 					= fs["Ixx"].as<double>();
	quadParam.Iyy 					= fs["Iyy"].as<double>();
	quadParam.Izz 					= fs["Izz"].as<double>();
	quadParam.maxVelocityRotors		= fs["maxVelocityRotors"].as<double>();

	// Устанавливаем параметры симулятора
	simParam.dt 					= fs["simulationStep"].as<double>();
	simParam.simulationTotalTime 	= fs["simulationTotalTime"].as<double>();
}

/**
 * @brief Чтение файла конфигурации системы управления ЛА,
 * 		установка параметров модели согласно конфигурации
 * @param filename - путь к файлу конфигурации
 * @param ControlSysParam - структура параметров системы управления
 */
void 	loadControlSysConfig(const std::string &fileName, ParamsControlSystem &controlSysParam)
{
	std::string	nameAxes = "XYZ";
	YAML::Node fs = YAML::LoadFile(fileName);
	
	// Устанавливаем параметры системы управления
	for	(unsigned int i = 0; i < 3; i++)
	{
		controlSysParam.KpAngularRate[i]	= fs[std::string("Kp") + nameAxes[i] + std::string("AngularRate")].as<double>();
		controlSysParam.KiAngularRate[i]	= fs[std::string("Ki") + nameAxes[i] + std::string("AngularRate")].as<double>();
		controlSysParam.KdAngularRate[i]	= fs[std::string("Kd") + nameAxes[i] + std::string("AngularRate")].as<double>();
		controlSysParam.KpAngle[i]			= fs[std::string("Kp") + nameAxes[i] + std::string("Angle")].as<double>();
		controlSysParam.KiAngle[i]			= fs[std::string("Ki") + nameAxes[i] + std::string("Angle")].as<double>();
		controlSysParam.KdAngle[i]			= fs[std::string("Kd") + nameAxes[i] + std::string("Angle")].as<double>();
		controlSysParam.KpPosition[i]		= fs[std::string("Kp") + nameAxes[i] + std::string("Position")].as<double>();
		controlSysParam.KiPosition[i]		= fs[std::string("Ki") + nameAxes[i] + std::string("Position")].as<double>();
		controlSysParam.KdPosition[i]		= fs[std::string("Kd") + nameAxes[i] + std::string("Position")].as<double>();
	}
	controlSysParam.maxCommandAngle					= fs["maxCommandAngle"].as<double>();
	controlSysParam.maxCommandAngularRate			= fs["maxCommandAngularRate"].as<double>();
	controlSysParam.maxCommandAngularAcceleration	= fs["maxCommandAngularAcceleration"].as<double>();
	controlSysParam.maxThrust						= fs["maxThrust"].as<double>();
}
