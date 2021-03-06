## *********************************************************
## 
## File autogenerated for the hearbo_cart_msgs package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'Int enum', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'int_enum_', 'edit_method': "{'enum_description': 'An enum to set the size.', 'enum': [{'srcline': 14, 'description': 'A small constant', 'srcfile': '../cfg/Test.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Small'}, {'srcline': 15, 'description': 'A medium value', 'srcfile': '../cfg/Test.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Medium'}, {'srcline': 16, 'description': 'A large value', 'srcfile': '../cfg/Test.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Large'}, {'srcline': 17, 'description': 'An extra large value', 'srcfile': '../cfg/Test.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'ExtraLarge'}]}", 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Int parameter', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'int_', 'edit_method': '', 'default': 0, 'level': 1, 'min': -10, 'type': 'int'}, {'srcline': 259, 'description': 'double parameter', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'double_', 'edit_method': '', 'default': 0.0, 'level': 2, 'min': -2.0, 'type': 'double'}, {'srcline': 259, 'description': 'String parameter', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'str_', 'edit_method': '', 'default': 'foo', 'level': 4, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Boolean parameter', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'bool_', 'edit_method': '', 'default': False, 'level': 8, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Contains the level of the previous change', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'level', 'edit_method': '', 'default': 0, 'level': 16, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'Checks against regression of #4499', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'int_nodefault', 'edit_method': '', 'default': 0, 'level': 0, 'min': -2147483648, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

Test_int_const = 5
Test_double_const = 5.6
Test_str_const = 'foo'
Test_bool_const = True
Test_Small = 0
Test_Medium = 1
Test_Large = 2
Test_ExtraLarge = 3
