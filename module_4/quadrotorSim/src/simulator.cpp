#include "simulator.hpp"

Simulator::Simulator(const ParamsQuadrotor &paramsQuadrotor,
				  const ParamsSimulator &paramsSimulator,
				  const ParamsControlSystem &paramsControlSystem)
{	
	// Объект математической модели БЛА
	mathModelQuadrotor = new MathModelQuadrotor(&paramsQuadrotor, &paramsSimulator);
	// Система планирования движения ЛА, осуществляет рассчет полиномов траектории
	// по массиву заданых точек(создается на усмотрение разработчика)
	// В самой простой интерпретации при достижении аппаратом окрестности
	// заданой точки отправляет новое пространственное положение на вход системы управления.
	// В случае создания оптимальной траектории рассчитывает полином положения от времени и
	// минимизирует траекторию согласно заданому критерию(см лекции)
	motionPlanner = new MotionPlanner();
	// объект системы управления БЛА
	controlSystem = new UAVControlSystem(&paramsControlSystem, &paramsSimulator, &paramsQuadrotor, motionPlanner);

	this->paramsSimulator = paramsSimulator;

	
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	// sock = socket(AF_INET, SOCK_DGRAM, 0);
	// Заполняем структуру для адресации при отправке пакетов
	memset((char *) &address, 0, sizeof(address));
	address.sin_family = AF_INET;
	//ставим номер порта
	address.sin_port = htons(PORT);

	slen=sizeof(address);
	//debug
	std::cout << "sim init" << std::endl;
}

Simulator::~Simulator()
{
	delete mathModelQuadrotor;
	delete motionPlanner;
	delete controlSystem;
}

/**
 * @brief основной метод запускающий процесс симуляции
 * 
 */
void Simulator::run()
{
	// Выполняем моделирование системы в цикле
	for (double t = 0; t < paramsSimulator.simulationTotalTime; t += paramsSimulator.dt)
	{
		// тут необходимо вызывать методы для получения комманд управления
		// тут необходимо вызывать методы для вычисления функции правых частей
		// математической модели, выполнять интегрирование приращений и формирование вектора состояния
		// Прим. Вектор состояния предлагается использовать в виде структуры(описание структуры в message.hpp)
		
		// Пример заполнения вектора состояния
		// Вектор состояния в результате должен формироваться по результатам интегрирования приращений
		// математической модели(интегрирования линейных и угловых ускорений)
		// Положение ЛА в стартовой СК
	    stateVector.X = 0; 
	    stateVector.Y = 0;
	    stateVector.Z = 3;
	    // Скорость ЛА в стартовой СК
	    stateVector.VelX = 0;
	    stateVector.VelY = 0;
	    stateVector.VelZ = 0;
	    // Угловое положение ЛА
	    stateVector.Pitch = 0;
	    stateVector.Roll = 0;
	    stateVector.Yaw = 0;
	    // Угловая скорость ЛА
	    stateVector.PitchRate = 0;
	    stateVector.RollRate = 0;
	    stateVector.YawRate = 0;
		// устанавливаем метку времени
		stateVector.timeStamp = t;
		// Отправляем вектор состояния
		sendMessage(stateVector);
		// Для простейшей имитации движения аппарата в реальном времени 
		// можно вызывать задержку или воспользоваться прерываниями
		usleep(paramsSimulator.dt * 1e6);
	}
}

void Simulator::sendMessage(const StateVector &stateVector)
{
	// Переведем вектор состояния
	if (sendto(sock, &stateVector, sizeof(stateVector) , 0 , (struct sockaddr *) &address, slen)==-1)
	{
		// выводим ошибку в терминал
		perror("sendto()");
		// завершаем выполнение программы
		exit(1);
	}
}
