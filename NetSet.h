#ifndef NETSET_H_
#define NETSET_H_
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include "oaDesignDB.h"

typedef std::vector<oa::oaPoint> Net_t;

// 0 stores VDD, 1 stores VSS, 2 stores S, 3 stores IO
typedef std::vector<std::vector<Net_t> > Nets_t;

typedef Nets_t::size_type NetType_t;

typedef Nets_t::value_type::const_iterator net_iterator;

const NetType_t VDD = 0;
const NetType_t VSS = 1;
const NetType_t S = 2;
const NetType_t IO = 3;

/*
class Netlist_t {
public:
     typedef std::pair<oa::oaPoint, oa::oaPoint> ContactPair_t;

     // read from netlist.txt and store netlist
     Netlist_t(std::ifstream &file);
     // return pairs of contacts to be routed
     void getContactPairs(std::vector<ContactPair_t> &pairs) const;

private:
     // connection info of the cell
     typedef unsigned char NetType_t;     // 0-VDD, 1-VSS, 2-S, 3-IO
     typedef std::vector<oa::oaPoint> _OneNet_t;
     typedef std::map<NetType_t, _OneNet_t> _Nets_t;

     // parse netlist.txt and extract one net
     NetType_t parseNetlist(_OneNet_t &net, const std::string &line);
     // store the net that just get parsed into internal container
     inline void addNetlist(NetType_t type, const _OneNet_t &net);

     _Nets_t _allNets;
};
*/

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
