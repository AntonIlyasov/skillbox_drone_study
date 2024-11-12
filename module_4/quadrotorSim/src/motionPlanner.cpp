#include "motionPlanner.hpp"

/**
 * @brief функция для расчёта траектории(заполняет коэффициенты полинома)
 * 
 * @param stateVector вектор состояния
 * @param targetPoints матрица координат точек (nx3), где n - кол-во точек
 * @param timeTrajectory массив времени, за которое требуется пролететь от точки до точки
 */
void			MotionPlanner::calculateTrajectory(StateVector stateVector, MatrixXd_t targetPoints, VectorXd_t timeTrajectory)
{
}

VectorXd_t		MotionPlanner::getRowsCoeffX(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(xCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffY(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(yCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffZ(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(zCoeff, indexRows));
}

double			MotionPlanner::getTimeTrajectory(unsigned int indexPoint)
{
	return (timeTrajectory(indexPoint));
}

unsigned int	MotionPlanner::getSizeTimeTrajectory()
{
	return (timeTrajectory.size());
}

/**
 * @brief функция для расчёта минимума по рывку
 * 
 * @param c 
 * @param t 
 * @return double 
 */
double			MotionPlanner::calculateDesiredPosition(VectorXd_t c, double t)
{
}

double			MotionPlanner::calculateDesiredVelocity(VectorXd_t c, double t)
{
}

double			MotionPlanner::calculateDesiredAcceleration(VectorXd_t c, double t)
{
}

MatrixXd_t		MotionPlanner::trajectoryGenerator(VectorXd_t currentPoints, VectorXd_t targetPoints, double T)
{
}