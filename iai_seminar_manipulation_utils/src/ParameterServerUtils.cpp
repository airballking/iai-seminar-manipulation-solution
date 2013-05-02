#include <iai_seminar_manipulation_utils/ParameterServerUtils.h>

bool loadStringVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<std::string>& parameters)
{
  XmlRpc::XmlRpcValue vector;
  if(!n.getParam(parameter_name, vector))
  {
    ROS_ERROR("Parameter '%s' not given in namespace '%s'",
      parameter_name.c_str(), n.getNamespace().c_str());
    return false;
  }

  if(vector.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Loaded parameter was not of type array: Parameter '%s' from namespace '%s'",
      parameter_name.c_str(), n.getNamespace().c_str());
    return false;
  }

  parameters.clear();
  for(int i=0; i<vector.size(); i++)
  {
    XmlRpc::XmlRpcValue &entry = vector[i];
    if(entry.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Entryi #%d in parameter was not of type string: Parameter '%s' from namespace '%s'", i,
        parameter_name.c_str(), n.getNamespace().c_str());
      return false;
    }

    parameters.push_back((std::string)entry);
  }

  // all went fine
  return true;
}

bool loadDoubleVectorFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  std::vector<double>& parameters)
{
  XmlRpc::XmlRpcValue vector;
  if(!n.getParam(parameter_name, vector))
  {
    ROS_ERROR("Parameter '%s' not given in namespace '%s'",
      parameter_name.c_str(), n.getNamespace().c_str());
    return false;
  }

  if(vector.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Loaded parameter was not of type array: Parameter '%s' from namespace '%s'",
      parameter_name.c_str(), n.getNamespace().c_str());
    return false;
  }

  parameters.clear();
  for(int i=0; i<vector.size(); i++)
  {
    XmlRpc::XmlRpcValue &entry = vector[i];
    if(entry.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR("Entryi #%d in parameter was not of type double: Parameter '%s' from namespace '%s'", i,
        parameter_name.c_str(), n.getNamespace().c_str());
      return false;
    }

    parameters.push_back((double)entry);
  }

  // all went fine
  return true;
}

bool loadDoubleFromParameterServer(ros::NodeHandle& n, const std::string& parameter_name,
  double& parameter)
{
  XmlRpc::XmlRpcValue param;
  if(!n.getParam(parameter_name, param))
  {
    ROS_ERROR("Parameter '%s' not given in namespace '%s'",
      parameter_name.c_str(), n.getNamespace().c_str());
    return false;
  }

  if(param.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    ROS_ERROR("Loaded parameter was not of type double: Parameter '%s' from namespace '%s'",
      parameter_name.c_str(), n.getNamespace().c_str());
    return false;
  }

  parameter = (double) param;

  // all went fine
  return true;
}
