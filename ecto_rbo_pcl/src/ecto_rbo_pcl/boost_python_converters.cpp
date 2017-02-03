/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/**
 * @brief
 *
 * A highly experimental set of converters to easily implicitly convert
 * pcl types from C++ to python cells.
 * The conversion target for python is a standard python list.
 *
 * Currently the following types are supported
 *  - indices (list of int) (most frequently used)
 *  - point cloud XYZ  (list of lists)
 *      (slow - data is copied!)
 *  - point cloud RGBXYZ (list of lists)
 *      (slow - data is copied!)
 *  - Some unaligned Eigen Matrix types and Transform
 *
 * If you want to test if conversion works open python and execute this code
 * <code>
 * import ecto_rbo_pcl_python
 * ecto_rbo_pcl_python.test_print_UnalignedVector4f([[0.25], [0.2], [0.35], [0]])
 * ecto_rbo_pcl_python.test_print_UnalignedVector4f([0.25, 0.2, 0.35, 0])
 * </code>
 *
 * It will invoke a C++ function which uses a converter.
 *
 * Known issues:
 *  - time stamp is not copied for point cloud
 *
 *
 *
 */

#include <Python.h>

#include <boost/python.hpp>
#include <boost/python/dict.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <string>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

namespace py = boost::python;


#include<vector>
#include<iostream>
#include<algorithm>
#include<iterator>

#include<boost/python.hpp>

template<typename T>
struct Vector_to_python_list
{

  static PyObject* convert(std::vector<T> const& v)
  {
    using namespace std;
    using namespace boost::python;
    using boost::python::list;
    list l;
    typename vector<T>::const_iterator p;
    for(p=v.begin();p!=v.end();++p){
      l.append(object(*p));
    }
    return incref(l.ptr());
  }
};


template<typename T>
struct Vector_from_python_list
{

    Vector_from_python_list()
    {
      using namespace boost::python;
      using namespace boost::python::converter;
      registry::push_back(&Vector_from_python_list<T>::convertible,
        &Vector_from_python_list<T>::construct,
        type_id<std::vector<T>
>());

    }

    // Determine if obj_ptr can be converted in a std::vector<T>
    static void* convertible(PyObject* obj_ptr)
    {
      if (!PyList_Check(obj_ptr) && !PyTuple_Check(obj_ptr)){
  return 0;
      }
      return obj_ptr;
    }

    // Convert obj_ptr into a std::vector<T>
    static void construct(
    PyObject* obj_ptr,
    boost::python::converter::rvalue_from_python_stage1_data* data)
    {
      using namespace boost::python;
      // Extract the character data from the python string
      //      const char* value = PyString_AsString(obj_ptr);

      boost::python::object* l;
      if (PyList_Check(obj_ptr)) {
        l = new list (handle<>(borrowed(obj_ptr)));
      } else if (PyTuple_Check(obj_ptr)){
        l = new tuple (handle<>(borrowed(obj_ptr)));;
      }

      // // Verify that obj_ptr is a string (should be ensured by convertible())
      // assert(value);

      // Grab pointer to memory into which to construct the new std::vector<T>
      void* storage = (
        (boost::python::converter::rvalue_from_python_storage<std::vector<T> >*)
        data)->storage.bytes;

      // in-place construct the new std::vector<T> using the character data
      // extraced from the python object
      std::vector<T>& v = *(new (storage) std::vector<T>());

      // populate the vector from list contains !!!
      int le = len(*l);
      v.resize(le);
      for(int i = 0;i!=le;++i){
        v[i] = extract<T>((*l)[i]);
      }

      // Stash the memory chunk pointer for later use by boost.python
      data->convertible = storage;

      delete l;
    }
};

template <class T>
void printVector(std::vector<T> v)
{
  using namespace std;
  copy(v.begin(),v.end(),
       ostream_iterator<T>(cout," "));
  cout << endl;
}



std::vector<std::vector<double> > test_generate_vectorVectorDouble()
{
  std::vector<std::vector<double> > p;
  for (int i=0; i < 5; i++) {
    std::vector<double> q;
    for (int j=0; j < 5; j++)
      q.push_back(i*j);
    p.push_back(q);
  }
  return p;
}

//////////////////////////////////////////////
// Indices

struct pclPointIndices_to_python_list
{
  static PyObject* convert(pcl::PointIndices const& indices)
  {
    //py::list l = std_vector_to_py_list(indices.indices);
    py::list l;
    for (size_t i = 0; i < indices.indices.size(); i++) {
      l.append(indices.indices[i]);
    }
    return boost::python::incref(l.ptr());
  }
};


struct pclPointIndices_from_python_list
{
  static void* convertible(PyObject* obj_ptr)  
  {
    if (PyList_Check(obj_ptr) || PyTuple_Check(obj_ptr))
      return obj_ptr;

    return 0;
  }

  static void construct(PyObject* obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    // Grab pointer to memory into which to construct the new pcl::PointIndices
    void* storage = ((boost::python::converter::rvalue_from_python_storage<pcl::PointIndices>*) data)->storage.bytes;
    
    // in-place construct the new QString using the character data
    // extraced from the python object
    new (storage) pcl::PointIndices;
    pcl::PointIndices* p = static_cast<pcl::PointIndices*>(storage);

    // Extract the character data from the python string
    Py_ssize_t size = PyList_Size(obj_ptr);
    //std::cout << "List size: " << size << std::endl;
    
    for (Py_ssize_t i = 0; i < size; i++) {
      PyObject* obj = PyList_GetItem(obj_ptr, i);
      assert (PyInt_Check(obj));
      int v = static_cast<int> (PyInt_AsLong(obj));
      //std::cout << " " << i << " -> " << v << std::endl;
      p->indices.push_back(v);
    }
    
    // Stash the memory chunk pointer for later use by boost.python
    data->convertible = storage;
  }
  
  pclPointIndices_from_python_list()
  {
    boost::python::converter::registry::push_back(
                                                  &convertible,
                                                  &construct,
                                                  boost::python::type_id<pcl::PointIndices>());
  }
};

/*
pcl::PointIndices test_generate_point_indices()
{
  pcl::PointIndices p;
  p.indices.push_back(1);
  p.indices.push_back(2);
  return p;
}

void test_print_point_indices(pcl::PointIndices p) {
  for (size_t i = 0; i < p.indices.size(); i++) {
    std::cout << p.indices[i] << std::endl;
  }
}
*/

//////////////////////////////////////////
// PointCloud to python

template <typename T>
void read_header(pcl::PointCloud<T> const& pc, py::dict& pc_dict) {
  // header
  py::dict header;
  header["frame_id"] = pc.header.frame_id;
  header["seq"] = pc.header.seq;
  // TODO need to write converter for ros::Time
//  header["stamp"] = pc.header.stamp;

  pc_dict["header"] = header;
}

struct pclPointCloudXYZ_to_python_list
{
  static PyObject* convert(pcl::PointCloud<pcl::PointXYZ> const& pc)
  {
    py::dict pc_dict;
    read_header<pcl::PointXYZ>(pc, pc_dict);

    // points
    py::list l;
    for (size_t i = 0; i < pc.points.size(); i++) {
      const pcl::PointXYZ &point = pc.points[i];
      py::list pointl;
      pointl.append(point.x);
      pointl.append(point.y);
      pointl.append(point.z);
      l.append(pointl);
    }
    pc_dict["points"] = l;

    return boost::python::incref(pc_dict.ptr());
  }
};

struct pclPointCloudXYZRGB_to_python_list
{
  static PyObject* convert(pcl::PointCloud<pcl::PointXYZRGB> const& pc)
  {
    py::dict pc_dict;
    read_header<pcl::PointXYZRGB>(pc, pc_dict);

    // points
    py::list l;
    for (size_t i = 0; i < pc.points.size(); i++) {
      const pcl::PointXYZRGB &point = pc.points[i];
      py::list pointl;
      pointl.append(point.x);
      pointl.append(point.y);
      pointl.append(point.z);
      pointl.append(point.r);
      pointl.append(point.g);
      pointl.append(point.b);
      l.append(pointl);
    }
    pc_dict["points"] = l;

    return boost::python::incref(pc_dict.ptr());
  }
};

/*
pcl::PointCloud<pcl::PointXYZRGB> test_generate_point_cloud()
{
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  pc.header.frame_id = "/world";
  pcl::PointXYZRGB p1, p2;
  p2.x = 1;
  pc.points.push_back(p1);
  pc.points.push_back(p2);
  return pc;
}
*/

//////////////////////////////////////////////
// Eigen Unaligned to python list of lists

typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> UnalignedAffine3f;

struct pclEigenUnalignedAffine3f_to_python_list
{
  static PyObject* convert(UnalignedAffine3f const& transform_)
  {
    Eigen::MatrixXf transform = transform_.matrix();

    //py::list l = std_vector_to_py_list(indices.indices);
    py::list l;
    for (size_t i = 0; i < transform.rows(); i++) {
      py::list m;
      for (size_t j = 0; j < transform.cols(); j++) {
        m.append(transform(i,j));
      }
      l.append(m);
    }
    return boost::python::incref(l.ptr());
  }
};

struct pclEigenUnalignedAffine3f_from_python_list
{
  static void* convertible(PyObject* obj_ptr)
  {
    if (!PyList_Check(obj_ptr) && !PyTuple_Check(obj_ptr))
      return 0;

    // TODO check dimensionality and that it is list of lists
    return obj_ptr;
  }

  static void construct(PyObject* obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    // Grab pointer to memory into which to construct the new pcl::PointIndices
    void* storage = ((boost::python::converter::rvalue_from_python_storage<UnalignedAffine3f>*) data)->storage.bytes;

    // in-place construct the new type using the data
    // extraced from the python object
    new (storage) UnalignedAffine3f;
    UnalignedAffine3f* p = static_cast<UnalignedAffine3f*>(storage);

    // Extract the character data from the python string
    Py_ssize_t size;
    if (PyList_Check(obj_ptr))
      size = PyList_Size(obj_ptr);
    else if (PyTuple_Check(obj_ptr))
      size = PyTuple_Size(obj_ptr);

    //std::cout << "List size: " << size << std::endl;

    for (Py_ssize_t i = 0; i < size; i++) {
      PyObject* list_ptr;
      assert (PyList_Check(list_ptr) || PyTuple_Check(list_ptr));

      if (PyList_Check(list_ptr))
          list_ptr = PyList_GetItem(list_ptr, i);
      else if (PyTuple_Check(list_ptr))
          list_ptr = PyTuple_GetItem(list_ptr, i);

      for (Py_ssize_t j = 0; j < PyList_Size(list_ptr); j++) {
        PyObject* obj;

        if (PyList_Check(obj))
            obj = PyList_GetItem(obj, i);
        else if (PyTuple_Check(obj))
            obj = PyTuple_GetItem(obj, i);

        double v;
        if (PyFloat_Check(obj)) {
          v = static_cast<float> (PyFloat_AsDouble(obj));
        }
        else if (PyInt_Check(obj)) {
          v = static_cast<float> (PyInt_AsLong(obj));
        } else {
          assert ("Cannot cast into numerical type" && false);
        }
        //std::cout << " (" << i << "," << j << ") -> " << v << std::endl;
        p->matrix()(i,j) = v;
      }
    }

    // Stash the memory chunk pointer for later use by boost.python
    data->convertible = storage;
  }

  pclEigenUnalignedAffine3f_from_python_list()
  {
    boost::python::converter::registry::push_back(
                                                  &convertible,
                                                  &construct,
                                                  boost::python::type_id<UnalignedAffine3f>());
  }
};

/*
UnalignedAffine3f test_generate_UnalignedAffine3f()
{
  UnalignedAffine3f p;
  for (int i=0; i < 4; i++)
    for (int j=0; j < 4; j++)
      p.matrix()(i,j)=0.;
  p.matrix()(0,0) = 1;
  p.matrix()(1,1) = 1;
  p.matrix()(2,2) = 1;
  p.matrix()(3,3) = 1;
  return p;
}

void test_print_UnalignedAffine3f(UnalignedAffine3f p) {
  std::cout << p.matrix() << std::endl;
}*/

//////////////////////////////////////////////
// Eigen Matrix to python list of lists

typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;

template<class EigenMatrixType>
struct pclEigenMatrix_to_python_list
{
  static PyObject* convert(EigenMatrixType const& transform)
  {
    //py::list l = std_vector_to_py_list(indices.indices);
    py::list l;
    for (size_t i = 0; i < transform.rows(); i++) {
      py::list m;
      for (size_t j = 0; j < transform.cols(); j++) {
        m.append(transform(i,j));
      }
      l.append(m);
    }
    return boost::python::incref(l.ptr());
  }
};

template<class EigenMatrixType>
struct pclEigenMatrix_from_python_list
{
  static void* convertible(PyObject* obj_ptr)
  {
    if (!PyList_Check(obj_ptr) && !PyTuple_Check(obj_ptr))
      return 0;

    // TODO check dimensionality and that it is list of lists
    return obj_ptr;
  }

  static void construct(PyObject* obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    // Grab pointer to memory into which to construct the new pcl::PointIndices
    void* storage = ((boost::python::converter::rvalue_from_python_storage<UnalignedAffine3f>*) data)->storage.bytes;

    // in-place construct the new type using the data
    // extraced from the python object
    new (storage) EigenMatrixType;
    EigenMatrixType* p = static_cast<EigenMatrixType*>(storage);

    // Extract the character data from the python string
    Py_ssize_t size = PyList_Size(obj_ptr);
    //std::cout << "List size: " << size << std::endl;

    for (Py_ssize_t i = 0; i < size; i++) {
      PyObject* list_ptr = PyList_GetItem(obj_ptr, i);

      if (PyList_Check(list_ptr) || PyTuple_Check(list_ptr)){
        Py_ssize_t size_elem = PyList_Size(list_ptr);
        assert(size_elem == p->cols());
        for (Py_ssize_t j = 0; j < size_elem; j++) {
          PyObject* obj;
          if (PyList_Check(list_ptr))
              obj = PyList_GetItem(list_ptr, j);
          else if (PyTuple_Check(list_ptr))
              obj = PyTuple_GetItem(list_ptr, j);

          double v;
          if (PyFloat_Check(obj)) {
            v = static_cast<float> (PyFloat_AsDouble(obj));
          }
          else if (PyInt_Check(obj)) {
            v = static_cast<float> (PyInt_AsLong(obj));
          } else {
            assert ("Cannot cast into numerical type" && false);
          }
          //std::cout << " (" << i << "," << j << ") -> " << v << std::endl;
          (*p)(i,j) = v;
        }
      } else {
        // flat list for vector type matrix?
        assert (p->cols() == 1 && size == p->rows());

        PyObject* obj = PyList_GetItem(obj_ptr, i);
        double v;
        if (PyFloat_Check(obj)) {
          v = static_cast<float> (PyFloat_AsDouble(obj));
        }
        else if (PyInt_Check(obj)) {
          v = static_cast<float> (PyInt_AsLong(obj));
        } else {
          assert ("Cannot cast into numerical type" && false);
        }
        //std::cout << " (" << i << "," << j << ") -> " << v << std::endl;
        (*p)(i,0) = v;
      }
    }

    // Stash the memory chunk pointer for later use by boost.python
    data->convertible = storage;
  }

  pclEigenMatrix_from_python_list()
  {
    boost::python::converter::registry::push_back(
                                                  &convertible,
                                                  &construct,
                                                  boost::python::type_id<EigenMatrixType>());
  }
};


UnalignedVector3f test_generate_UnalignedVector3f()
{
  UnalignedVector3f p;
  for (int i=0; i < 1; i++)
    for (int j=0; j < 3; j++)
      p(j,i)=0.;
  p(0,0) = 1;
  return p;
}

void test_print_UnalignedVector3f(UnalignedVector3f p) {
  std::cout << p << std::endl;
}

std::vector<UnalignedVector3f> test_generate_VectorUnalignedVector3f()
{
  std::vector<UnalignedVector3f> res;
  for (int i = 0; i < 10; i++) {
    UnalignedVector3f p;
    for (int i=0; i < 1; i++)
      for (int j=0; j < 3; j++)
        p(j,i)=0.;
    p(0,0) = 1;
    res.push_back(p);
  }
  return res;
}



UnalignedVector4f test_generate_UnalignedVector4f()
{
  UnalignedVector4f p;
  for (int i=0; i < 1; i++)
    for (int j=0; j < 5; j++)
      p(j,i)=0.;
  p(0,0) = 1;
  return p;
}

void test_print_UnalignedVector4f(UnalignedVector4f p) {
  std::cout << p << std::endl;
}


//////////////////////////////////////////////

BOOST_PYTHON_MODULE(ecto_rbo_pcl_python)
{
  // register the to-python converter for PointIndices
  py::to_python_converter<pcl::PointIndices, pclPointIndices_to_python_list>();
  // register the from-python converter
  pclPointIndices_from_python_list();
  // testing
  //py::def("test_generate_point_indices", test_generate_point_indices);
  //py::def("test_print_point_indices", test_print_point_indices);

//  py::def("test_generate_point_cloud", test_generate_point_cloud);
  // register the to-python converter for PointCloud
  py::to_python_converter<pcl::PointCloud<pcl::PointXYZ>, pclPointCloudXYZ_to_python_list>();
  py::to_python_converter<pcl::PointCloud<pcl::PointXYZRGB>, pclPointCloudXYZRGB_to_python_list>();
  
  // register the converters for UnalignedAffine3f
  py::to_python_converter<UnalignedAffine3f, pclEigenUnalignedAffine3f_to_python_list>();
  pclEigenUnalignedAffine3f_from_python_list();
  py::to_python_converter<std::vector<UnalignedAffine3f>, Vector_to_python_list<UnalignedAffine3f> >();
  Vector_from_python_list<UnalignedAffine3f>();
  // testing
  //py::def("test_generate_UnalignedAffine3f", test_generate_UnalignedAffine3f);
  //py::def("test_print_UnalignedAffine3f", test_print_UnalignedAffine3f);

  //to-Python converter for std::vector<int, std::allocator<int> > already registered
  //py::to_python_converter<std::vector<int>, Vector_to_python_list<int> >();
  //py::to_python_converter<std::vector<double>, Vector_to_python_list<double> >();

  // register the converters for simple vector types
  Vector_from_python_list<std::string>();
  Vector_from_python_list<int>();
  Vector_from_python_list<double>();
  py::to_python_converter<std::vector<std::vector<double> >, Vector_to_python_list<std::vector<double> > >();
  Vector_from_python_list<std::vector<double> >();
  py::def("test_generate_vectorVectorDouble", test_generate_vectorVectorDouble);
  //py::def("test_print_vectorVectorDouble", printVector<std::vector<double> >);
  py::def("test_print_vectorInt", printVector<int>);

  // register the converters for Eigen::Matrix types
  // UnalignedVector3f
  py::to_python_converter<UnalignedVector3f, pclEigenMatrix_to_python_list<UnalignedVector3f> >();
  pclEigenMatrix_from_python_list<UnalignedVector3f>();
  // vector types
  py::to_python_converter<std::vector<UnalignedVector3f>, Vector_to_python_list<UnalignedVector3f> >();
  Vector_from_python_list<UnalignedVector3f>();
  // testing
  py::def("test_generate_UnalignedVector3f", test_generate_UnalignedVector3f);
  py::def("test_print_UnalignedVector3f", test_print_UnalignedVector3f);
  py::def("test_generate_VectorUnalignedVector3f", test_generate_VectorUnalignedVector3f);
  py::def("test_print_VectorUnalignedVector3f", printVector<UnalignedVector3f>);

  // register the to-python converter

  // UnalignedVector4f
  py::to_python_converter<UnalignedVector4f, pclEigenMatrix_to_python_list<UnalignedVector4f> >();
  pclEigenMatrix_from_python_list<UnalignedVector4f>();
  // vector types
  py::to_python_converter<std::vector<UnalignedVector4f>, Vector_to_python_list<UnalignedVector4f> >();
  Vector_from_python_list<UnalignedVector4f>();
  // testing
  py::def("test_generate_UnalignedVector4f", test_generate_UnalignedVector4f);
  py::def("test_print_UnalignedVector4f", test_print_UnalignedVector4f);

  // UnalignedMatrix3f
  py::to_python_converter<UnalignedMatrix3f, pclEigenMatrix_to_python_list<UnalignedMatrix3f> >();
  pclEigenMatrix_from_python_list<UnalignedMatrix3f>();
  // vector types
  py::to_python_converter<std::vector<UnalignedMatrix3f>, Vector_to_python_list<UnalignedMatrix3f> >();
  Vector_from_python_list<UnalignedMatrix3f>();
}
