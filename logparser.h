#ifndef LOGPARSER_H
#define LOGPARSER_H

#include <vector>
#include <fstream>
#include <string>
#include "define.h"

class LogParser
{
public:
    LogParser();
    bool loadLog(std::string rawptfilename, std::string featfilename);

    std::vector<point4d> getRawptbuf() const;

    std::vector<linefeat> getLinefeatbuf() const;

    std::vector<planefeat> getPlanefeatbuf() const;

private:
    std::vector <point4d> rawptbuf;
    std::vector <linefeat> linefeatbuf;
    std::vector <planefeat> planefeatbuf;

};

#endif // LOGPARSER_H
