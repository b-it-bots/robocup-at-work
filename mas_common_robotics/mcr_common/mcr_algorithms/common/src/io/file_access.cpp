/*
 * file_access.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: Frederik Hegger
 */

#include "mcr_algorithms/io/file_access.h"

int FileAccess::writeVectorToFile(std::vector<std::vector<double> > &input_vector, std::string filename, std::string separator)
{
    //open output file
    std::ofstream file_output;
    file_output.open(filename.c_str(), std::ios::trunc);

    // open sample files
    if (!file_output.is_open())
    {
        std::cout << "cannot open sample file: " << filename << std::endl;
        return -1;
    }

    for (unsigned int i = 0; i < input_vector.size(); ++i)
    {
        for (unsigned int j = 0; j < input_vector[i].size(); ++j)
        {
            if (j == (input_vector[i].size() - 1))
                file_output << input_vector[i][j] << std::endl;
            else
                file_output << input_vector[i][j] << separator;
        }
    }

    file_output.close();

    return 0;
}
