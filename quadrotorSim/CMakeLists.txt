cmake_minimum_required(VERSION 3.10)
project(uavMathModel)

# Устанавливаем настройки компилятора
set(CMAKE_CXX_STANDARD 17 )
set(COMMON_FLAGS "-g")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}")

# Находим пути к файлам библиотек 
find_package (Eigen3  REQUIRED)
find_package (Threads)

# устанавливаем пути к заголовочным файлам
include_directories(
		inc
 		${CMAKE_CURRENT_SOURCE_DIR}
 		${EIGEN3_INCLUDE_DIR}
	)
# Создаем переменную для файлов реализации
set(SIM_CPP_FILES 
	src/mathModelQuadrotor.cpp
	src/simulator.cpp
	src/UAVControlSystem.cpp
	src/math.cpp
	src/configLoader.cpp
	src/motionPlanner.cpp
)
# Добавляем библиотеку для симулятора БЛА
add_library(multicoter_simulator STATIC ${SIM_CPP_FILES})
# Добавляем исполняемый файл
add_executable(main src/main.cpp)
# Выполняем подключение библиотек к исполняемому файлу
target_link_libraries(main multicoter_simulator yaml-cpp ${CMAKE_THREAD_LIBS_INIT})