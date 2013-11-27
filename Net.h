#ifndef NET_H_
#define NET_H_

#include <vector>
#include "oaDesignDB.h"
#include "RouterConfig.h"

class Net_t : public std::vector<oa::oaPoint> {
public:
    Net_t(const std::vector<oa::oaPoint> &points, oa::oaUInt4 id, NetType_t type, \
            oa::oaString portName="")
        : std::vector<oa::oaPoint>(points), _id(id), _type(type), \
          _portName(portName) {}

    oa::oaInt4 id() const { return _id; }
    NetType_t type() const { return _type; }
    oa::oaString portName() const { return _portName; }
private:
    oa::oaInt4 _id;
    NetType_t _type;
    oa::oaString _portName;
};

#endif
