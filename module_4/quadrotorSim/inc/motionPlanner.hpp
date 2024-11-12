#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <typesData.hpp>
#include <math.hpp>
#include <message.hpp>

class MotionPlanner
{
	public:
		void			calculateTrajectory(StateVector stateVector, MatrixXd_t targetPoints, VectorXd_t timeTrajectory);
		VectorXd_t		getRowsCoeffX(unsigned int indexRows);
		VectorXd_t		getRowsCoeffY(unsigned int indexRows);
		VectorXd_t		getRowsCoeffZ(unsigned int indexRows);
		unsigned int	getSizeTimeTrajectory();
		double			getTimeTrajectory(unsigned int indexPoint);
		double			calculateDesiredPosition(VectorXd_t c, double t);
		double			calculateDesiredVelocity(VectorXd_t c, double t);
		double			calculateDesiredAcceleration(VectorXd_t c, double t);
		
	private:
		// Массив коэффициентов для каждой точки(строка - коэффициенты для точки)
		MatrixXd_t	xCoeff;
		MatrixXd_t	yCoeff;
		MatrixXd_t	zCoeff;
		// промежутки времени за которое БЛА пролетает траекторию
		VectorXd_t	timeTrajectory;

		MatrixXd_t		trajectoryGenerator(VectorXd_t currentPoints, VectorXd_t targetPoints, double T);
};

#endif