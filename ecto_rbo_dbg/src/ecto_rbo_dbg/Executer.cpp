/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto/ecto.hpp>
#include <ecto/python.hpp>
#include <boost/foreach.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler.hpp>

#include "ecto_rbo_yaml/PlasmYamlFactory.h"

using namespace ecto;

namespace ecto_rbo_dbg
{

struct Executer
{
    typedef scheduler scheduler_t;
    //    static void
    //    shallow_merge(const tendrils& fts, tendrils& ts)
    //    {
    //        std::string key;
    //        tendril_ptr t;
    //        BOOST_FOREACH(boost::tie(key,t), fts)
    //        {
    //            t->required(false);
    //            ts.declare(key, t);
    //        }
    //    }

    static void declare_params(ecto::tendrils& params)
    {
        plasm_.reset(new ecto::plasm);
        PlasmYamlFactory factory(plasm_);
        factory.load("/home/clemens/catkin_ws/src/ecto_rbo/ecto_rbo_yaml/data/simple_grasp_sampler.yaml");

        PlasmYamlFactory::CellsByName& cells = factory.getCellsByName();
        for (PlasmYamlFactory::CellsByName::iterator it = cells.begin(); it != cells.end(); ++it)
        {
            for (ecto::tendrils::const_iterator jt = it->inputs.begin(); jt != it->inputs.end(); ++jt)
            {
                std::cerr << " DF " << jt->type_id() << std::endl;
            }
        }

        //        params.declare<>
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        //        inputs.declare<ecto::pcl::Clusters>("cluster_0", "Cluster that should be part of the merger.").required(true);
        //        for (int i = 1; i < max_input_messages; ++i)
        //        {
        //            std::string name = "cluster_" + boost::lexical_cast<std::string>(i);
        //            inputs.declare<ecto::pcl::Clusters>(name, "Cluster that should be part of the merger.").required(false);
        //        }

        //        outputs.declare<ecto::pcl::Clusters>(&MergeClusters::merged_clusters_, "merged_clusters", "Merged clusters.");

        //        shallow_merge(parmas, base->parameters);
        //        shallow_merge(inputs, base->inputs);
        //        shallow_merge(o, base->outputs);
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        //        clusters_.resize(max_input_messages);
        //        for (int i = 0; i < max_input_messages; ++i)
        //        {
        //            std::string name = "cluster_" + boost::lexical_cast<std::string>(i);
        //            clusters_[i] = inputs[name];
        //        }
    }

    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
        //FIXME
        //TODO Scheduler is a pain here. Need to expose as a scope so that exceptions are informative.
        if (!sched_)
        {
            try
            {
                plasm_->configure_all();

            } catch (ecto::except::EctoException& e)
            {
                throw std::runtime_error(ecto::except::diagnostic_string(e));
            }
            sched_.reset(new scheduler_t(plasm_));
        }
        try
        {
            if (niter_ > 0)
                sched_->execute(niter_);
            else
                sched_->execute(0);
            if (! sched_->running())
                return ecto::QUIT;
        } catch (ecto::except::EctoException& e)
        {
            throw std::runtime_error(ecto::except::diagnostic_string(e));
        }
        return ecto::OK;
    }

    ecto::plasm::ptr plasm_;
    boost::shared_ptr<scheduler_t> sched_;
    int niter_;
};
}

ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::Executer, "ecto_rbo_dbg::Executer", "Executes a plasm.");
