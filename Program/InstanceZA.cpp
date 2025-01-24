//
// Created by chkwon on 3/22/22.
//

#include <fstream>
#include <cmath>
#include <iostream>
#include <sstream>
#include "InstanceZA.h"

InstanceZA::InstanceZA(std::string pathToInstance, bool isRoundingInteger = true)
{
    std::string content, content2, content3;
    double serviceTimeData = 0.;

    // Read INPUT dataset
    std::ifstream inputFile(pathToInstance);
    if (inputFile.is_open())
    {
        int n;

        //double vehicleCapacity;

        // Reading node coordinates
        // depot must be the first element
        // 		- i = 0 in the for-loop below, or
        // 		- node_number = 1 in the .vrp file
        // customers are
        // 		- i = 1, 2, ..., nbClients in the for-loop below, or
        // 		- node_number = 2, 3, ..., nb_Clients in the .vrp file

        // Calculating 2D Euclidean Distance
        std::string line;
        while (std::getline(inputFile, line))
        {

            // Handle `n`
            if (line.find("n =") != std::string::npos)
            {
                n = std::stoi(line.substr(line.find('=') + 1));
                nbClients = n - 1;
            }
            // Handle `t`
            else if (line.find("t =") != std::string::npos)
            {
                dist_mtx = std::vector<std::vector<double>>(nbClients + 1, std::vector<double>(nbClients + 1));
                std::string matrix = line.substr(line.find('[') + 1);
                matrix.pop_back(); // Remove the closing ']'
                std::stringstream ss(matrix);
                std::string row;
                dist_mtx.clear();
                while (std::getline(ss, row, ';'))
                {
                    std::vector<double> t_row;
                    std::stringstream row_stream(row);
                    double value;
                    while (row_stream >> value)
                    {
                        t_row.push_back(value);
                    }
                    dist_mtx.push_back(t_row);
                }
            }
            // Handle `th`
            else if (line.find("th =") != std::string::npos)
            {
                th = std::vector<double>(nbClients + 1);

                std::string vector_data = line.substr(line.find('[') + 1);
                vector_data.pop_back(); // Remove the closing ']'
                std::stringstream ss(vector_data);
                double value;
                th.clear();
                while (ss >> value)
                {
                    th.push_back(value);
                    if (ss.peek() == ',')
                        ss.ignore(); // Skip commas
                }
            }
            // Handle `T`
            else if (line.find("T =") != std::string::npos)
            {
                T = std::stod(line.substr(line.find('=') + 1));
            }
            // Handle `d`
            else if (line.find("d =") != std::string::npos)
            {
                demands = std::vector<double>(nbClients + 1);
                service_time = std::vector<double>(nbClients + 1);

                std::string vector_data = line.substr(line.find('[') + 1);
                vector_data.pop_back(); // Remove the closing ']'
                std::stringstream ss(vector_data);
                double value;
                demands.clear();
                while (ss >> value)
                {
                    demands.push_back(value);
                    service_time.push_back(0.);

                    if (ss.peek() == ',')
                        ss.ignore(); // Skip commas
                }
            }
            // Handle `C`
            else if (line.find("C =") != std::string::npos)
            {
                
                vehicleCapacity = std::stod(line.substr(line.find('=') + 1));
            }
        }
    }
    else
        throw std::string("Impossible to open instance file: " + pathToInstance);
}
