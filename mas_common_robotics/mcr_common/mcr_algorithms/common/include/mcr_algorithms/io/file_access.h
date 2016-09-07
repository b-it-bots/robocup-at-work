/*
 * file_access.h
 *
 *  Created on: Apr 19, 2011
 *      Author: Frederik Hegger
 */

#ifndef FILE_ACCESS_H_
#define FILE_ACCESS_H_

#include <vector>
#include <fstream>
#include <iostream>

class FileAccess
{
public:
    static int writeVectorToFile(std::vector<std::vector<double> > &input_vector, std::string filename, std::string separator);
};

#endif /* FILE_ACCESS_H_ */
