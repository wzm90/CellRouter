#ifndef NETSET_H_
#define NETSET_H_
#include <string>
#include <fstream>
#include "oaDesignDB.h"
#include "RouterConfig.h"

const NetType_t VDD = 0;
const NetType_t VSS = 1;
const NetType_t S = 2;
const NetType_t IO = 3;


class NetSet_t {
public:
    NetSet_t(std::ifstream &file);
    net_iterator begin(NetType_t type) const  { return _allNets[type].begin(); }
    net_iterator end(NetType_t type) const { return _allNets[type].end(); }
private:
    NetType_t parseNet(Net_t &net, const std::string &line);
    inline void addNet(NetType_t type, const Net_t &net); 
    
    Nets_t _allNets;
};

inline void NetSet_t::addNet(NetType_t type, const Net_t &net) {
    _allNets[type].push_back(net);
}


#endif
