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
    // route one net;
    bool routeOneNet(NetType_t type, Net_t &net);
private:
    oa::oaDesign *_design;
    oa::oaTech *_tech;
    oa::oaBox _VDDBox;
    oa::oaBox _VSSBox;
    NetSet_t _nets;
    DRC_t _designRule;
    
};
#endif
