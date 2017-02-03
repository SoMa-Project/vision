/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/*
 * PlasmYamlFactoryRViz.h
 *
 *  Created on: Aug 9, 2016
 *      Author: clemens
 */

#ifndef PLASMYAMLFACTORYRVIZ_H_
#define PLASMYAMLFACTORYRVIZ_H_

#include "ecto_rbo_yaml/PlasmYamlFactory.h"

#include <string>
#include <vector>

#include <ecto/ecto.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/plasm.hpp>

class PlasmYamlFactoryRViz: public PlasmYamlFactory
{
public:
    PlasmYamlFactoryRViz(const ecto::plasm_ptr& plasm);
    virtual ~PlasmYamlFactoryRViz();

    void addRVizCellsToOutputs(const std::vector<std::string>& cell_names);

private:
    std::string randomString(size_t length);
    void addCell(const std::string& type, const ecto::cell_ptr& parent, const std::string& parent_output, const ecto::cell_ptr& child, const std::string& child_input);
};

#endif /* PLASMYAMLFACTORYRVIZ_H_ */
