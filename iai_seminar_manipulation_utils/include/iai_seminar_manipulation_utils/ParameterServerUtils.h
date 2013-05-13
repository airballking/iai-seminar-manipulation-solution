#ifndef PARAMETERSERVERUTILS_H_
#define PARAMETERSERVERUTILS_H_

#include <ros/ros.h>
#include <vector>

/** Loads a vector of strings from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter holding the vector.
    \param parameters [out] container for loaded vector.
    \return [out] true if everything worked out fine, else false.
*/
bool loadStringVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<std::string>& parameters);

/** Loads a vector of doubles from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter holding the vector.
    \param parameters [out] container for loaded vector.
    \return [out] true if everything worked out fine, else false.
*/
bool loadDoubleVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<double>& parameters);

/** Loads a single double from the parameter server.

    \param n [in] namespace to use for the lookup.
    \param parameter_name [in] name of the parameter.
    \param parameter [out] container for loaded double.
    \return [out] true if everything worked out fine, else false.
*/
bool loadDoubleFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  double& parameter);

#endif /* PARAMETERSERVERUTILS_H_ */
