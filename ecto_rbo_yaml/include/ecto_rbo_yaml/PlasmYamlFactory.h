/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/*
 * PlasmYamlFactory.h
 *
 *  Created on: Nov 26, 2012
 *      Author: clemens
 */

#ifndef PLASMYAMLFACTORY_H_
#define PLASMYAMLFACTORY_H_

#include <string>
#include <vector>
#include <map>

#include <ecto/python.hpp>
#include <ecto/ecto.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/plasm.hpp>

namespace YAML { class Node; }

class PlasmYamlFactory
{
public:
  // this type holds the name of the cell, parameter, type and range
  // e.g. (("ListenTF", "rate"), ("double_t", "0, 20"))
  typedef std::map<std::pair<std::string, std::string>, std::pair<std::string, std::string> > DynamicParams;
  typedef std::map<std::string, ecto::cell_ptr> CellsByName;

public:
  PlasmYamlFactory(const ecto::plasm_ptr& plasm);
  virtual ~PlasmYamlFactory();

  void addToPythonPath(const std::string& path);
  void setPythonPath(const std::vector<std::string>& path);
  std::vector<std::string> getPythonPath();

  void addModule(const std::string& module);

  void save(const std::string& name);
  void load(const std::string& name);

  CellsByName& getCellsByName();
  DynamicParams& getDynamicParameters();

protected:
  static const char* const DYNAMIC_RECONFIGURE_TAG;

  ecto::plasm_ptr plasm;
  CellsByName cells_by_name;
  DynamicParams dynamic_parameters;

  std::vector<std::string> python_path;
  std::vector<std::string> import_modules;

  std::vector<std::string> loadDynamicReconfigureParameters(const YAML::Node* node);
};

#endif /* PLASMYAMLFACTORY_H_ */
