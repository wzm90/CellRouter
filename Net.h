#include <vector>
#include "oaDesignDB.h"

class Net_t : public std::vector<oa::oaPoint> {
public:
    Net_t() : _portName("") {}
    Net_t(const oa::oaString &name) : _portName(name) {}
    oa::oaString portName() const { return _portName; }
    void setPortName(const oa::oaString &name) { _portName = name; }
private:
    oa::oaString _portName;
};
