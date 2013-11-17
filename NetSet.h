#ifndef NETSET_H_
#define NETSET_H_
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include "oaDesignDB.h"
#include "Net.h"

//typedef std::vector<oa::oaPoint> Net_t;

// 0 stores VDD, 1 stores VSS, 2 stores S, 3 stores IO
typedef std::vector<std::vector<Net_t> > Nets_t;

typedef Nets_t::size_type NetType_t;

typedef Nets_t::value_type::const_iterator net_iterator;

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
