#ifndef ROUTER_H_
#define ROUTER_H_

#include "oaDesignDB.h"
#include "NetSet.h"
#include "DRC.h"

class Router_t {
public:
    Router_t(oa::oaDesign *design, oa::oaTech *tech, std::ifstream &file1,\
            std::ifstream &file2);
    bool route();
private:
    // route one net;
    bool routeOneNet(NetType_t type, Net_t &net);
    bool routeVDD(Net_t &net);
    bool routeVSS(Net_t &net);
    bool routeSignal(Net_t &net);
    bool routeIO(Net_t &net);
    void createWire(const oa::oaPoint &lhs, const oa::oaPoint &rhs);

    oa::oaDesign *_design;
    oa::oaTech *_tech;
    oa::oaBox _VDDBox;
    oa::oaBox _VSSBox;
    NetSet_t _nets;
    DRC_t _designRule;
    
};
#endif
