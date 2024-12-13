#ifndef CONFIGLOADER_HPP
#define CONFIGLOADER_HPP

#include <typesData.hpp>
#include <yaml-cpp/yaml.h>

void	loadModelConfig(const std::string &fileName, ParamsQuadrotor &quadParam, ParamsSimulator &simParam);
void 	loadControlSysConfig(const std::string &fileName, ParamsControlSystem &controlSysParam);


#endif
