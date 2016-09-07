/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#include "logger.h"

CLogger* CLogger::instance = 0;

CLogger& CLogger::getInstance()
{
    if (!instance)
    {
        std::string loggerCfgFilePath = std::string(LOGGERCONFIG);
        instance = new CLogger(loggerCfgFilePath);
    }
    return *instance;
}

CLogger& CLogger::getInstance(std::string path)
{
    if (!instance)
    {
        std::string loggerCfgFilePath = path + std::string(LOGGERCONFIG);
        instance = new CLogger(loggerCfgFilePath);
    }
    return *instance;
}

void CLogger::destroy()
{
    if (instance)
    {
        delete instance;
    }
    instance = 0;
}

std::vector<std::string> CLogger::loadLoggerConfig(std::string loggerConfigFile)
{
    std::fstream fin;
    std::string priority;
    std::string output;
    std::vector < std::string > configString;
    std::cout << "CLogger::loadLoggerConfig... " << loggerConfigFile << "\n";

    fin.open(std::string(loggerConfigFile).c_str(), std::ios::in | std::ios::binary);

    if (fin.is_open())
    {
        std::string temp;

        fin >> temp;
        fin >> priority;
        //std::cout << temp << " " << priority << "\n";
        configString.push_back(priority);

        fin >> temp;
        fin >> output;
        //std::cout << temp << " " << output << "\n";
        configString.push_back(output);

        fin.close();
    }
    else
    {
        std::cout << "CLogger::loadLoggerConfig...could not open " << loggerConfigFile << "\n";
    }

    return configString;

}

void CLogger::deleteLogFile(std::string fileName)
{
    std::fstream fin(fileName.c_str(), std::ios::in);
    if (fin)
    {
        fin.close();
        fin.open(fileName.c_str(), std::ios::out | std::ios::trunc);
        if (fin)
        {
            std::cout << "CLogger::deleteLogFile...Done!" << "\n";
            fin.close();
        }

    }
}
