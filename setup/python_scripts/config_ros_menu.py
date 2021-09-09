#!/bin/usr/python3
#
#Copyright (c) 2021 Wei-Chih Lin
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

import os, sys, re
from ruamel.yaml import YAML
import argparse, getpass

class CONFIG_YAML(object):
    """
    Default configuration:
      - project: ROSKY
      - ros_version: 2 
    """
    def __init__(self, ros_version=1):
    
        # get project
        self.project = re.split('/', os.getcwd())
        self.project = self.project[len(self.project) - 3]  # find project name
        
        # set common function
        self.yaml = YAML() 
    
        #setup argument
        self.shell = os.popen("echo $SHELL | awk -F '/' '{print $NF}'").readlines()[0].rstrip("\n")
        self.home_path = f"/home/" + os.popen("echo $USER | awk -F '/' '{print $NF}'").readlines()[0].rstrip("\n")
        self.ubuntu = os.popen("grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}'").readlines()[0].rstrip("\n")
        
        ## use inter function
        self.ros_distro = self.get_ros_distro(ros_version=ros_version)
        
    def read(self, _file):
        with open(_file) as _file:
            content = self.yaml.load(_file)
        return content 
        
    def write(self, content, _file):
        """
        write(content, _file)
        """
        with open(_file, "w") as outfile:
            self.yaml.preserve_quotes = False
            self.yaml.dump(content, outfile)
        print(f"Done! Now you can open \"{_file}\" to check out!")
            
    def get_ros_distro(self, ros_version=1):
        ros_distro_list = self.read(self.get_path(folder="/setup/env_variables", _file="ros_distro.yaml")) 
        return f"ROS {ros_version} " + ros_distro_list[f"ROS{ros_version}"][self.ubuntu]
		
    def get_path(self, project="", folder="setup", _file="environment.sh"):
        """
        get_path(project=String, folder=String, _file=String)
        """
        project = self.project if project == "" else project
        return f"{self.home_path}/{project}/{folder}/{_file}".replace("//", "/")
        
if __name__ == '__main__':

    # call class
    config = CONFIG_YAML()

    # configure argument
    target_file = config.get_path(project="ros_menu", folder="", _file="config.yaml")
    #test_file = config.get_path(_file="test.yaml")
    parser = argparse.ArgumentParser()
    
    ## ros version
    parser.add_argument("--ros-version", 
            type=int, 
            help=f"Please set ros version that you want to add command in file \"{target_file}\"", 
            default=2)
            
    ## cmd    
    parser.add_argument("--cmd", 
            type=str, 
            help="source environment.sh", 
            default="source ~/ROSKY2/setup/shell_scripts/environment.sh")
            
    args = parser.parse_args()

    # read content
    content = config.read(target_file)
    
    # configure cmd
    cmd = args.cmd
    if content["Menu"][config.ros_distro]["cmds"] is None:
        content["Menu"][config.ros_distro]["cmds"] = [cmd]
    else:
        content["Menu"][config.ros_distro]["cmds"].insert(0, cmd)
    
    # write content
    config.write(content, target_file)
    

     	


