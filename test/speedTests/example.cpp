#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}

// py::list addToList_Python(py::list myList) {
//     for(int i = 0; i < py::len(myList); i++ )
//         myList[i] = myList[i] + py::float_(5.0);

//     return myList;
// }

std::vector<double> addToList_Cpp(std::vector<double> &myList) {
    for(int i = 0; i < myList.size(); i++ )
        myList[i] += 5.0;

    return myList;
}

py::array_t<double> addFive_Loop(py::array_t<double> myList) {
    auto buf = myList.request();
    double* ptr = (double*) buf.ptr;
    for(int i = 0; i < buf.size; i++ )
        ptr[i] = ptr[i] + 5.0;

    return myList;
}

double addFive(double a) {
    return a + 5.0;
}

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");
    
    m.def("addToList_Cpp", &addToList_Cpp, "A function to add 5 to every element in an C++ vector, converted from a Python list");
    
    m.def("addFive", &addFive_Loop);

    m.def("vectorizedAddFive", py::vectorize(addFive));
}