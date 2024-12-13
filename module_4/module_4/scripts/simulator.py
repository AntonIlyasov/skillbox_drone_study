import sys
sys.path.append('/home/anton202/skillbox_drone_study/module_4/module_4/src')
import multicopter_model.constants as cs
from multicopter_model.controller import QuadCopterController
from multicopter_model.model import QuadCopterModel
from multicopter_model.simulator import Simulator

def main():
	# Создадим экземпляр класса контроллера
	controller = QuadCopterController()
	# Инициализируем параметры системы управления, коэффициенты нужно настроить самостоятельно.

	# POSITION CONTROLLER
	controller.position_controller_x.set_pid_gains(5, 1, 0)
	controller.position_controller_x.set_saturation_limit(-3, 3)
	controller.position_controller_x.integral_limit = 0

	controller.position_controller_y.set_pid_gains(5, 1, 0)
	controller.position_controller_y.set_saturation_limit(-3, 3)
	controller.position_controller_y.integral_limit = 0

	controller.position_controller_z.set_pid_gains(5, 1, 0)
	controller.position_controller_z.set_saturation_limit(-3, 3)
	controller.position_controller_z.integral_limit = 0

	# VELOCITY CONTROLLER
	controller.velocity_controller_x.set_pid_gains(0.1, 1, 0)
	controller.velocity_controller_x.set_saturation_limit(-0.5, 0.5)
	controller.velocity_controller_x.integral_limit = 0.0

	controller.velocity_controller_y.set_pid_gains(0.1, 1, 0)
	controller.velocity_controller_y.set_saturation_limit(-0.5, 0.5)
	controller.velocity_controller_y.integral_limit = 0.0

	controller.velocity_controller_z.set_pid_gains(50, 0, 0)
	controller.velocity_controller_z.set_saturation_limit(-25, 25)
	controller.velocity_controller_z.integral_limit = 0.0

	# ANGLE CONTROLLER
	controller.roll_controller.set_pid_gains(4, 1, 1)
	controller.roll_controller.set_saturation_limit(-5, 5)
	controller.roll_controller.integral_limit = 0.0
      
	controller.pitch_controller.set_pid_gains(4, 0, 1)
	controller.pitch_controller.set_saturation_limit(-5, 5)
	controller.pitch_controller.integral_limit = 0.0

	controller.yaw_controller.set_pid_gains(4, 0, 0.5)
	controller.yaw_controller.set_saturation_limit(-5, 5)
	controller.yaw_controller.integral_limit = 0.0

	# ANGULAR RATE CONTROLLER
	controller.roll_rate_controller.set_pid_gains(150, 0, 0)
	controller.roll_rate_controller.set_saturation_limit(-150, 150)
	controller.roll_rate_controller.integral_limit = 0.0

	controller.pitch_rate_controller.set_pid_gains(150, 0, 0)
	controller.pitch_rate_controller.set_saturation_limit(-150, 150)
	controller.pitch_rate_controller.integral_limit = 0.0

	controller.yaw_rate_controller.set_pid_gains(50, 0, 0)
	controller.yaw_rate_controller.set_saturation_limit(-20, 20)
	controller.yaw_rate_controller.integral_limit = 0.0

	# Создаем модель мультикоптера и запускаем симуляцию
	model = QuadCopterModel(cs.quadcopter_inertia, cs.quadcopter_mass,
								cs.trust_coef, cs.drag_coef, cs.arm_length)

	sim = Simulator(controller, model, 0.01)
	sim.run()

if __name__ == "__main__":
    main()