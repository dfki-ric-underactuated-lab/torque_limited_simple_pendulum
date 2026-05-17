#ifndef UTILS_HPP
#define UTILS_HPP

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>


class CSVReader{
    std::string fileName;
    std::string delimeter;
public:
    CSVReader(std::string filename, std::string delm = ",") :
            fileName(filename), delimeter(delm)
    { }
    // Function to fetch data from a CSV File
    std::vector<std::vector<std::string> > getData(int);
    std::vector<std::vector<double> > getDataDouble(int);
    void writeHeader(std::string header);
    void saveData(std::vector<std::vector<double> > data);
    void saveData(std::vector<int> data);
    void saveData(std::vector<long unsigned int> data);
    void saveData(std::vector<long int> data);
};

bool contains(std::vector<std::size_t> const&vals, int val);
template<typename T>
inline void remove(std::vector<T> &v, const T & item);
#endif // UTILS_HPP
