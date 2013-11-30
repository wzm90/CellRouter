#ifndef NET_H_
#define NET_H_

#include <vector>
#include "oaDesignDB.h"
#include "RouterType.h"

class Net_t : public std::vector<oa::oaPoint> {
public:
    Net_t(const std::vector<oa::oaPoint> &points, oa::oaUInt4 id, NetType_t type, \
            oa::oaString portName="");

    oa::oaInt4 id() const { return _id; }
    NetType_t type() const { return _type; }
    oa::oaString portName() const { return _portName; }
    oa::oaBoolean contains(const oa::oaPoint &point, \
            oa::oaBoolean incEdge=true) { return _bbox.contains(point, incEdge); }
private:
    oa::oaInt4 _id;
    NetType_t _type;
    oa::oaString _portName;
    oa::oaBox _bbox;    // bounding box of the net, used in net ordering
};

#endif
