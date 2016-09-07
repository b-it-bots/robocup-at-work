/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef CLOGGER_H_
#define CLOGGER_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

#define LOG4CPP_FIX_ERROR_COLLISION 1
#include <log4cpp/Category.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/SimpleLayout.hh>

#define LOGFILE "log_object_categorization.txt"
#define LOGGERCONFIG "ros/config/logger_cfg.txt"

#define PRIORITY DEBUG
//#define PRIORITY INFO
//#define PRIORITY WARN
//#define PRIORITY ERROR

class CLogger
{
private:
    log4cpp::Appender *appender;
    log4cpp::Layout *layout;
    static CLogger* instance;

    std::vector<std::string> loadLoggerConfig(std::string loggerConfigFile);

    CLogger(std::string loggerCfgFilePath)
    {
        std::string output;
        std::string priority;
        std::vector < std::string > configString;
        configString = loadLoggerConfig(loggerCfgFilePath);

        priority = configString[0];
        output = configString[1];
        //For files::
        if (output.compare("cout") == 0)
        {
            std::cout << "CLogger...output set to " << output << "\n";
            appender = new log4cpp::OstreamAppender("OstreamAppender", &std::cout);
        }
        else
        {
            std::cout << "CLogger...output set to File : " << output << "\n";
            CLogger::deleteLogFile(output);
            appender = new log4cpp::FileAppender("FileAppender", (char*) output.c_str());
        }

        layout = new log4cpp::SimpleLayout();
        log4cpp::Category& category = log4cpp::Category::getInstance("Category");

        appender->setLayout(layout);
        category.setAppender(appender);

        if (priority.compare("DEBUG") == 0)
        {
            std::cout << "CLogger...Debug level set to " << priority << "\n";
            category.setPriority(log4cpp::Priority::DEBUG);
        }
        else if (priority.compare("INFO") == 0)
        {
            std::cout << "CLogger...Debug level set to " << priority << "\n";
            category.setPriority(log4cpp::Priority::INFO);
        }
        else if (priority.compare("WARN") == 0)
        {
            std::cout << "CLogger...Debug level set to " << priority << "\n";
            category.setPriority(log4cpp::Priority::WARN);
        }
        else if (priority.compare("ERROR") == 0)
        {
            std::cout << "CLogger...Debug level set to " << priority << "\n";
            category.setPriority(log4cpp::Priority::ERROR);
        }
        else
        {
            std::cout << "CLogger...Debug level set to default: DEBUG \n";
            category.setPriority(log4cpp::Priority::DEBUG);
        }
        log = &category;
    }
    CLogger(const CLogger&)
    {
    }
    ~CLogger()
    {
        delete appender;
        delete layout;
        delete log;
    }
public:
    log4cpp::Category *log;
    static CLogger& getInstance();
    static CLogger& getInstance(std::string path);
    static void destroy();
    static void deleteLogFile(std::string fileName);
    //static void setLoggerPath(std::string path);
};

#endif
