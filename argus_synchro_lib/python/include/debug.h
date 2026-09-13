#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <string>
#include <tuple>
#include <vector>

namespace py = pybind11;

inline void print_matrix(const Eigen::MatrixXd& mat)
{
    std::cout << "(" << mat.rows() << ", " << mat.cols() << ")\n";
    std::cout << "[";
    for (int i = 0; i < mat.rows(); ++i)
    {
        std::cout << "[";
        for (int j = 0; j < mat.cols(); ++j)
        {
            std::cout << mat(i, j);
            if (j < mat.cols() - 1)
                std::cout << " ";
        }
        std::cout << "]";
        if (i < mat.rows() - 1)
            std::cout << "\n";
    }
    std::cout << "]" << std::endl;
}

template <typename T> std::string array_info(py::array_t<T> array)
{
    auto ndim = array.ndim();
    auto shape = std::vector<int>();
    std::string s = "(";
    for (auto dim = 0; dim < ndim - 1; dim++)
    {
        auto len = array.shape(dim);
        shape.push_back(len);
        s = s.append(std::to_string(len));
        s = s.append(",");
    }
    shape.push_back(array.shape(ndim - 1));
    s = s.append(std::to_string(array.shape(ndim - 1)));
    s = s.append(")\n");

    if (ndim == 1)
    {
        s = s.append("[");
        for (auto i = 0; i < shape[0] - 1; i++)
        {
            s = s.append(std::to_string(*array.data(i)));
            s = s.append(" ");
        }
        s = s.append(std::to_string(*array.data(shape[0] - 1)));
        s = s.append("]");
    }
    else if (ndim == 2)
    {
        s = s.append("[");
        for (auto i = 0; i < shape[0] - 1; i++)
        {
            s = s.append("[");
            for (auto j = 0; j < shape[1] - 1; j++)
            {
                s = s.append(std::to_string(*array.data(i, j)));
                s = s.append(" ");
            }
            s = s.append(std::to_string(*array.data(i, shape[1] - 1)));
            s = s.append("]\n");
        }
        s = s.append("[");
        for (auto j = 0; j < shape[1] - 1; j++)
        {
            s = s.append(std::to_string(*array.data(shape[0] - 1, j)));
            s = s.append(" ");
        }
        s = s.append(std::to_string(*array.data(shape[0] - 1, shape[1] - 1)));
        s = s.append("]]");
    }

    return s;
};

template <typename T> void print_array_info(py::array_t<T> array)
{
    std::cout << array_info(array) << std::endl;
};

template <typename T> std::string map_info(std::map<std::tuple<int, int, int>, T> map)
{
    std::string s = "(" + std::to_string(map.size()) + "," + std::to_string(3) + ")\n";

    s = s.append("[");
    auto it = map.begin();
    for (auto i = 0; i < map.size() - 1; i++)
    {
        s = s.append("(");
        s = s.append(std::to_string(std::get<0>(it->first)));
        s = s.append(",");
        s = s.append(std::to_string(std::get<1>(it->first)));
        s = s.append(",");
        s = s.append(std::to_string(std::get<2>(it->first)));
        s = s.append(")\n");
        it++;
    }
    s = s.append("(");
    s = s.append(std::to_string(std::get<0>(it->first)));
    s = s.append(",");
    s = s.append(std::to_string(std::get<1>(it->first)));
    s = s.append(",");
    s = s.append(std::to_string(std::get<2>(it->first)));
    s = s.append(")]");

    return s;
};

template <typename T> inline void print_map_info(std::map<std::tuple<int, int, int>, T> map)
{
    std::cout << map_info(map) << std::endl;
};

template <typename T> inline void print_tuple(std::tuple<T, T, T> t)
{
    std::cout << std::get<0>(t) << "," << std::get<1>(t) << "," << std::get<2>(t) << std::endl;
}
