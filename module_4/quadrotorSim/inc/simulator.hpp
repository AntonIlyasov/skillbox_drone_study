#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include	<typesData.hpp>
#include	<sys/socket.h>
#include	<arpa/inet.h>
#include	<iostream>
#include	<string>
#include 	<mathModelQuadrotor.hpp>
#include 	<UAVControlSystem.hpp>
#include 	<message.hpp>
#include 	<math.hpp>
#include 	<unistd.h>
#include 	<chrono>
// адрес сервера на который мы будем отправлять пакеты
#define SERVER "127.0.0.1" //localhost
#define PORT 12346	//Порт отправки данных



class Simulator
{
	private:
		int 				sock;
		int 				slen;
		struct 				sockaddr_in address;
		double  			simulationTime;
		double				simulationStep;
		MathModelQuadrotor 	*mathModelQuadrotor;
		UAVControlSystem   	*controlSystem;
		MotionPlanner		*motionPlanner;
		StateVector			stateVector;
		ParamsSimulator     paramsSimulator;
		
	public:
		Simulator(const ParamsQuadrotor 	&paramsQuadrotor,
				  const ParamsSimulator 	&paramsSimulator,
				  const ParamsControlSystem &paramsControlSystem);
		~Simulator();
		void run();
		void sendMessage(const StateVector &stateVector);
};




#endif 

