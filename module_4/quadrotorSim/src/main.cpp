#include 	<simulator.hpp>
#include 	<typesData.hpp>
#include 	<configLoader.hpp>


int	main()
{
	// создаем экземпляры структур под параметры симулятора
	ParamsQuadrotor		parmsQuadrotor;
	ParamsSimulator		paramsSimulator;
	ParamsControlSystem	paramsControlSystem;

	// заполнение параметров квадрокоптера и симулятора
	loadModelConfig(pathQuadModelConfig, parmsQuadrotor, paramsSimulator);

	// заполнение параметров
	loadControlSysConfig(pathQuadControlSystemConfig, paramsControlSystem);

	// Создаем объект симулятора
	Simulator	uavSim(parmsQuadrotor, paramsSimulator, paramsControlSystem);
	// Запускаем симуляцию
	uavSim.run();


	return (0);
}