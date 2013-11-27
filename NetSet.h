#ifndef NETSET_H_
#define NETSET_H_
#include <string>
#include <fstream>
#include "oaDesignDB.h"
#include "Net.h"
#include "RouterConfig.h"

class NetSet_t : public std::vector<Net_t> {
public:
    NetSet_t(std::ifstream &file);
private:
    // parse one line of input text, initialize a net and add it into NetSet
    void parseAddNet(const std::string &line);
};

#endif
