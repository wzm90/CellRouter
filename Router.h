#ifndef ROUTER_H_
#define ROUTER_H_

#include <vector>
#include "oaDesignDB.h"
#include "Net.h"
#include "NetSet.h"
#include "DRC.h"
#include "EndPoint.h"

class Router_t {
public:
    Router_t(oa::oaDesign *design, oa::oaTech *tech, std::ifstream &file1,\
            std::ifstream &file2);
    bool route();
private:
    typedef enum { LEFT, BOTTOM, RIGHT, TOP } CoverType;
    // BarrierSet_t: containters for storing line barriers, 
    // used in line-probing algorithm
    typedef std::multimap<oa::oaCoord, std::pair<oa::oaInt4, line_t> > BarrierSet_t;
    
    void reorderNets();
    bool routeOneNet(const Net_t &net);
    bool routeVDD(const Net_t &net);
    bool routeVSS(const Net_t &net);
    bool routeSignal(const Net_t &net);
    bool routeIO(const Net_t &net);
    void createWire(const oa::oaPoint &lhs, const oa::oaPoint &rhs, oa::oaInt4 netID);
    void createVia(const oa::oaPoint &point, oa::oaInt4 netID);
    bool routeTwoContacts(EndPoint_t &lhs, EndPoint_t &rhs);
    // escape: perform escape algorithm
    bool escape(EndPoint_t &src, EndPoint_t &dst, oa::oaPoint &intersectionPoint);
    void getEscapeLine(const EndPoint_t &src, Orient_t orient, line_t &escapeLine);
    void getEscapePoint(EndPoint_t &src); 
    void getCover(const EndPoint_t &src, CoverType type, line_t &cover);
    void addObstacle(oa::oaLayerNum layer, oa::oaInt4 netID, const oa::oaBox &box);

    line_t &lineSeg(const BarrierSet_t::iterator &it) {return (it->second).second;}
    oa::oaInt4 netID(const BarrierSet_t::iterator &it) {return (it->second).first;}
    oa::oaCoord coord(const BarrierSet_t::iterator &it) {return it->first;}

    oa::oaDesign *_design;
    oa::oaTech *_tech;
    oa::oaBox _VDDBox;
    oa::oaBox _VSSBox;
    NetSet_t _nets;
    DRC_t _designRule;
    BarrierSet_t _m1Barriers;
    BarrierSet_t _m2Barriers;
};
#endif
