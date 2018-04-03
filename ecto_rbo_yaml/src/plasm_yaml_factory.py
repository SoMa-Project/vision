import ecto
import ecto_pcl
import ecto_rbo_pcl
import ecto_rbo_pcl_python
#import ecto_rbo_ml_py
import ecto_rbo_grasping
import ecto_ros
import ecto_opencv.imgproc
import ecto_opencv.highgui
import ecto_ros.ecto_sensor_msgs
import ecto_ros.ecto_std_msgs
import ecto_ros.ecto_geometry_msgs
import ecto_rbo_dbg_py
import ecto_rbo_dbg


import datetime

import rospkg
rospack = rospkg.RosPack()

import yaml
import sys
import os
import random, string

import re
import numpy
import numpy as np

modules = {}


def randomword(length):
   return ''.join(random.choice(string.lowercase) for i in xrange(length))

# collect all available modules
for module in [ecto_pcl, ecto_rbo_pcl, ecto_rbo_grasping, ecto_ros.ecto_sensor_msgs, ecto_ros.ecto_std_msgs, ecto_ros.ecto_geometry_msgs, ecto_opencv.imgproc, ecto_opencv.highgui, ecto_ros]:
    for name, clazz in module.__dict__.iteritems():
        if name[:2] == "__":
            continue
        mod_name = module.__name__
        if "." in mod_name:
          # add once with full module name
          mod_name_full = mod_name.replace(".", "::")
          modules[(mod_name_full, name)] = clazz
          mod_name = mod_name.split(".")[-1]
        modules[(mod_name, name)] = clazz

#----------------------------------------------------------------------

def load(yml_files, debug = False):
    """ Load an ecto plasm and build the plasm from a yaml file or a set of yaml files.
    
    Possible parameters:
      - string containing path to yaml file
      - list of strings containing paths to yaml files
      - dictionary
      
    The factory also supports magic parameters. A magic parameter contains arbitrary python code
    that is executed during deparsing. To write a magic parameter just put eval(..) around it, e.g.
      myparam: eval(np.mean([1,2,3,4]))
    """

    dc = {}

    if type(yml_files) == dict:
      dc = yml_files
    elif type(yml_files) == str:
      with open(yml_files, "r") as f:
        dc = yaml.load(f)
    elif type(yml_files) == list or type(yml_files) == tuple:
      for yml_file in yml_files:
        with open(yml_file, "r") as f:
          dc_cur = yaml.load(f)
        # check that now key is overwritten
        for k in dc_cur.keys():
          if k in dc:
            raise Exception ("Key %s is defined in at least to yml files (e.g. in %s)" % (k, yml_file) )
        dc.update(dc_cur)

    return build_plasm_from_dictionary(dc, debug)

#----------------------------------------------------------------------
def load_from_string(yml_str):
  dc = yaml.load(yml_str)
  return build_plasm_from_dictionary(dc)
#----------------------------------------------------------------------

def build_plasm_from_dictionary(yml_file, debug = False):

    # build cells
    cells = {}
    cells_inputs = {}
    for cell_name,cell_dict in yml_file.iteritems():
      cell_type = cell_dict['type']
      del cell_dict['type']
      
      try:
        cell_inputs = cell_dict['inputs']
        del cell_dict['inputs']
        cells_inputs[cell_name] = cell_inputs
      except:
        cell_inputs = None

      cell_params = dict([ (p[1:],v) for p,v in cell_dict.iteritems() if p[0] == "$" ])
      cell_params = dict(cell_params, ** dict([ (p,v) for p,v in cell_dict.iteritems() if p[0] != "$"]))
      
      # eval magic params
      for k,v in cell_params.iteritems():
        try:
          v = v.strip()
          if v[:5]=="eval(" and v[-1] == ")":
            cell_params[k] = eval(v[5:-1])
            print ("    %s: Evaluating magic param %s -> %s" % (cell_name, k, str(cell_params[k])))
        except:
          pass
      
      #clean up cell_type
      cell_type = cell_type.strip()
      res = re.search("ecto::pcl::PclCell(WithNormals|DualInputs)?<([a-zA-Z0-9\_:]+)>", cell_type)
      if res:
        cell_type = res.group(2)
      res = re.search("(imgproc::Filter_)?<([a-zA-Z0-9\_:]+)>", cell_type)
      if res:
        cell_type = res.group(2)
      cell_type = cell_type.replace("ecto::pcl", "ecto_pcl")
       
      for (pkg, mod), clazz in modules.iteritems():
        cell_class_name = "%s::%s" % (pkg, mod)
        
        if cell_class_name == cell_type:
          if cell_name in cells:
              print cells
              print ("Error: cell with same name already included: %s (type %s, %s)" % (cell_name, cell_type, cell_class_name))
              raise Exception()
            
          # sanity check: are cell_params actually params?
          cell_params_iter = cell_params.copy()
          try:
            for p, _ in cell_params_iter.iteritems():
                if not clazz.params.has_key(p):
                  print ("Warn: cell %s does not have key %s" % (cell_name, p))
                  del cell_params[p]              
          except AttributeError as e:
            print ("  Warn: querying params for %s failed. Probably a python cell? Make sure you specified the correct params" % cell_name)
          
          if debug:
            print (" Adding %s (type: %s, class: %s)" % (cell_name, cell_type, cell_class_name))
          try:
            cells[cell_name] = clazz(cell_name, **cell_params)
          except Exception as e:
            print ("  Unable to instantiate cell %s of type %s with params: %s" % (cell_name, cell_class_name, str(cell_params)))
            raise e
      
      if cell_name not in cells:
          print (cells)
          raise Exception("Error: cell not found: %s for %s" % (cell_type, cell_name))
    
    #print cells
    #print cells_inputs
            
    # connect cells
    plasm = ecto.Plasm()
    connection = []
    for cell_name, cell_inputs in cells_inputs.iteritems():
      for input_tendril_name, output_string in cell_inputs.iteritems():
        try:
          output_cell_name, output_tendril_name = output_string.split("/")
        except ValueError:
          raise Exception("Malformatted input for cell '%s': %s \n(expecting 'input_cell_name/tendril')" % (cell_name, output_string))
        if debug:
          print (" Connecting %s.%s >> %s.%s" % (output_cell_name, output_tendril_name, cell_name, input_tendril_name))
        try:
          in_tendril = cells[output_cell_name][output_tendril_name]
        except KeyError as e:
          raise Exception("Cannot find input tendril: %s.%s - do cell and tendril exist?" % (output_cell_name, output_tendril_name))
        try:
          out_tendril = cells[cell_name][input_tendril_name]
        except KeyError as e:
          raise Exception("Cannot find output tendril: %s.%s - do cell and tendril exist?" % (cell_name, input_tendril_name))
        
        connection.append ( in_tendril >> out_tendril ) 

    if debug:
        # go through all dbg cells
        outputs_to_dbg_cell = {}
        cells_that_need_aux_input = []
        for module in [ecto_rbo_dbg, ecto_rbo_dbg_py]:
                    for name, clazz in module.__dict__.iteritems():
                            if name[:2] == "__" or (name[:3].islower() if ('__all__' in module.__dict__) else False):
                                    continue
                            try:
                                    instance = clazz()
                                    type_of_dbg_input = instance.inputs.at('input').type_name
                                    outputs_to_dbg_cell[type_of_dbg_input] = clazz
                                    if len(instance.inputs) > 1:
                                            cells_that_need_aux_input.append(clazz)
                            except ecto.NonExistant as e:
                                    pass

        # go through all output types
        new_cells = {}

        # add markerarray publisher cell
        rviz_pub_cell = None
        rviz_pub_cell_input_counter = 0
        #if (len(new_cells) > len(cells_that_need_aux_input)):
        if True:
            cell_name = "rviz_publisher_%s" % (randomword(7))
            rviz_pub_cell = ecto_rbo_dbg.RVizMarkerPublisher(cell_name)
            new_cells[cell_name] = rviz_pub_cell
            
            for name, cell in cells.iteritems():
                if cell.type().endswith('Message2PointCloud'):
                    connection.append( cell['header'] >> rviz_pub_cell['header'] )
                    break

        # connect debug cells whereever possible
        for name, cell in cells.iteritems():
                    for output_tendril_name in cell.outputs.keys():
                            outputtype = cell.outputs.at(output_tendril_name).type_name
                            if outputtype in outputs_to_dbg_cell:
                                    #cells[cell_name] = clazz(**cell_params)
                                    cell_name = "rviz_publisher_%s" % (randomword(7))
                                    cell_params = {"namespace": "%s/%s" % (name, output_tendril_name)}
                                    new_cell = outputs_to_dbg_cell[outputtype](cell_name, **cell_params)
                                    new_cells[cell_name] = new_cell

                                    connection.append( cell[output_tendril_name] >> new_cell['input'] )
                                    
                                    # check if more inputs are needed
                                    if outputs_to_dbg_cell[outputtype] in cells_that_need_aux_input:
                                        # debug cell needs a second input
                                        # find cell that feeds cell['input'] (type pointcloud)
                                        try:
                                            aux_output_cell_name, aux_output_tendril_name = cells_inputs[name]['input'].split('/')
                                            connection.append( cells[aux_output_cell_name][aux_output_tendril_name] >> new_cells[cell_name]['pointcloud'] )
                                        except Exception as e:
                                            # this exception is mainly for cells whose input is not a pointcloud e.g. MergeClusters
                                            print("Couldn't add debugging cell to visualize output of cell {}.".format(name))
                                            del new_cells[cell_name]
                                            del new_cell
                                            del connection[-1]
                                            #print e
                                    
                                    # check for outputs
                                    try:
                                        if len(new_cell.outputs) > 0:
                                            # debug cell creates a marker message that will be aggregated by the rviz_pub_cell
                                            connection.append( new_cell['output'] >> rviz_pub_cell["input_%i" % (rviz_pub_cell_input_counter)] )
                                            rviz_pub_cell_input_counter += 1
                                    except NameError:
                                        # it was deleted before
                                        pass
        cells.update(new_cells)
    
    plasm.connect(connection)
    return plasm, cells
    
#print load("/home/shoefer/apc_workspace/src/ecto_rbo/ecto_rbo_yaml/data/amazon_bin_features.yaml")
