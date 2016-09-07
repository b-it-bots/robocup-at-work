/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <assert.h>

#include "file_settings.h"



std::map<int, std::string> CFileSettings::trainPath;
std::map<int, std::string> CFileSettings::labels;

void CFileSettings::coutStdVector(std::vector<double> data, bool newLines)
{
    if (newLines)
    {
        std::cout << "\n";
    }

    for (unsigned int i = 0; i < data.size(); ++i)
    {
        std::cout << data[i] << " ";
    }

    if (newLines)
    {
        std::cout << "\n";
    }

}
void CFileSettings::saveLibSvmItem(bool isAppend, std::string fileName,
                                   std::vector<int> labels, std::vector<std::vector<double> > features)
{
    std::fstream fout;

    if (!isAppend) // do not append
    {
        fout.open(std::string(fileName).c_str(), std::ios::out
                  | std::ios::binary);
    }
    else   //append
    {
        fout.open(std::string(fileName).c_str(), std::ios::out
                  | std::ios::binary | std::ios::app);
    }

    assert(features.size() > 0);
    assert(features.size() == labels.size());

    std::cout << "Saving... NOW  " << fileName << std::endl;
    for (unsigned int i = 0; i < features.size(); ++i)
    {
        fout << labels[i] << " ";
        std::cout << labels[i] << " ";
        for (unsigned int j = 0; j < features[i].size(); ++j)
        {
            int no = j + 1;
            fout << no << ":" << features[i][j] << " ";
            std::cout << no << ":" << features[i][j] << " ";
        }
        fout << std::endl;
        std::cout << std::endl;
    }
    fout.close();
    //} else {
    //  std::cout << "Could not save libsvm file!!!" << fileName << "\n";
    //}

}

void CFileSettings::saveLibSvmItem2(bool isAppend, std::string fileName,
                                    std::vector<int> labels, std::vector<std::vector<double> > features)
{
    std::fstream fout;

    if (!isAppend) // do not append
    {
        fout.open(std::string(fileName).c_str(), std::ios::out
                  | std::ios::binary);
    }
    else   //append
    {
        fout.open(std::string(fileName).c_str(), std::ios::out
                  | std::ios::binary | std::ios::app);
    }

    assert(features.size() > 0);
    assert(features.size() == labels.size());

    std::cout << "Saving... NOW  " << fileName << std::endl;
    for (unsigned int i = 0; i < features.size(); ++i)
    {
        fout << labels[i] << " ";
        std::cout << labels[i] << " ";
        for (unsigned int j = 0; j < features[i].size(); ++j)
        {
            int no = j + 1;
            fout << no << ":" << features[i][j] << " ";
            std::cout << no << ":" << features[i][j] << " ";
        }
        fout << std::endl;
        std::cout << std::endl;
    }
    fout.close();
    //} else {
    //  std::cout << "Could not save libsvm file!!!" << fileName << "\n";
    //}

}

void CFileSettings::saveUnsupervisedItem(bool isAppend, std::string fileName, std::map<int, std::vector<double> > features)
{
    std::vector<std::vector<double> > vecfeatures;
    std::map<int, std::vector<double> >::iterator iterFeatures;

    for (iterFeatures = features.begin(); iterFeatures != features.end(); ++iterFeatures)
    {
        vecfeatures.push_back(iterFeatures->second);
    }
    saveUnsupervisedItem(isAppend, fileName, vecfeatures);
}

void CFileSettings::saveUnsupervisedItem(bool isAppend, std::string fileName, std::vector<std::vector<double> > features)
{
    std::fstream fout;

    if (features.size() == 0)
    {
        std::cout << "CFileSettings::saveUnsupervisedItem...nothing to save  features are empty!\n";
    }


    if (!isAppend) // do not append
    {
        fout.open(std::string(fileName).c_str(), std::ios::out
                  | std::ios::binary);
    }
    else   //append
    {
        fout.open(std::string(fileName).c_str(), std::ios::out
                  | std::ios::binary | std::ios::app);
    }
    //if (fout.is_open()) {

    assert(features.size() > 0 && features[0].size() > 0);

    std::cout << "CFileSettings::saveUnsupervisedItem...Saving... " << fileName << "\n";
    fout << features.size() << " " << features[0].size() << "\n";
    for (unsigned int i = 0; i < features.size(); ++i)
    {
        for (unsigned int j = 0; j < features[i].size(); ++j)
        {
            if (j == 0)
            {
                fout << features[i][j];
            }
            else
            {
                fout << " " << features[i][j];
            }
        }
        fout << "\n";
    }
    fout.close();
    //} else {
    //  std::cout << "Could not save libsvm file!!!" << fileName << "\n";
    //}

}


void CFileSettings::saveStdVector(std::string fileName,
                                  std::vector<double> data)
{

    std::fstream fout;

    std::cout << "Saving... " << fileName << "\n";
    fout.open(fileName.c_str(), std::ios::out | std::ios::binary);

    fout << 1 << "\n" << data.size() << "\n";

    for (unsigned int j = 0; j < data.size(); ++j)
    {
        fout << data[j] << " ";
    }
    fout << "\n";

    fout.close();
}

std::vector<std::vector<double> > CFileSettings::loadStdVector(
    std::string fileName)
{

    std::fstream fin;
    std::vector<std::vector<double> > readVector;
    std::cout << "Loading... " << fileName << "\n";

    fin.open(std::string(fileName).c_str(), std::ios::in | std::ios::binary);

    if (fin.is_open())
    {

        unsigned int numVec;
        unsigned int dim;
        double temp;

        fin >> numVec;
        fin >> dim;

        for (unsigned int i = 0; i < numVec; ++i)
        {
            std::vector<double> data;
            for (unsigned int j = 0; j < dim; ++j)
            {

                fin >> temp;
                data.push_back(temp);
            }
            readVector.push_back(data);
        }
        fin.close();
    }
    else
    {
        std::cout << "Could not read " << fileName << "\n";
    }

    return readVector;
}

void CFileSettings::tokenize(const std::string& str,
                             std::vector<std::string>& tokens, const std::string& delimiters)
{
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
}

