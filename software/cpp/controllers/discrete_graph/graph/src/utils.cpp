#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>

#include "utils.hpp"

std::vector<std::vector<std::string> > CSVReader::getData(){
/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
    std::ifstream file(fileName);
    std::vector<std::vector<std::string> > dataList;
    std::string line = "";
    // Iterate through each line and split the content using delimeter
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        dataList.push_back(vec);
    }
    // Close the File
    file.close();
    return dataList;
}

void CSVReader::writeHeader(std::string header){
    std::ofstream file;
    file.open(fileName);
    file << header;
    file << "\n";
    file.close();
}

void CSVReader::saveData(std::vector<std::vector<double> > data){
    //std::cout << "saveData called\n";
    std::ofstream file;
    file.open(fileName, std::ios::app);
    for(int line=0; line<data.size(); line++){
        for(int cell=0; cell<data[line].size(); cell++){
            file << data[line][cell];
            if(cell != data[line].size() - 1){
                file << ",";
            }
        }
        file << "\n";
    }
    file.close();
}

void CSVReader::saveData(std::vector<int> data){
    //std::cout << "saveData called\n";
    std::ofstream file;
    file.open(fileName, std::ios::app);
    for(int line=0; line<data.size(); line++){
        file << data[line];
        file << "\n";
    }
    file.close();
}

void CSVReader::saveData(std::vector<long unsigned int> data){
    //std::cout << "saveData called\n";
    std::ofstream file;
    file.open(fileName, std::ios::app);
    for(int line=0; line<data.size(); line++){
        file << data[line];
        file << "\n";
    }
    file.close();
}

void CSVReader::saveData(std::vector<long int> data){
    //std::cout << "saveData called\n";
    std::ofstream file;
    file.open(fileName, std::ios::app);
    for(int line=0; line<data.size(); line++){
        file << data[line];
        file << "\n";
    }
    file.close();
}

bool contains(std::vector<std::size_t> const& vals, int val)
{
    return std::count(std::begin(vals), std::end(vals), val) != 0;
}

template<typename T>
inline void remove(std::vector<T> & v, const T & item)
{
    v.erase(std::remove(v.begin(), v.end(), item), v.end());
}
