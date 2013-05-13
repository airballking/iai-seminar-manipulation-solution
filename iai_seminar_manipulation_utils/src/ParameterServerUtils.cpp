/*  Copyright (c) 2013, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *   
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the Institute for Artificial Intelligence/Universität Bremen
 *      nor the names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
