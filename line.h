#ifndef LINE_H_
#define LINE_H_

#include <utility>
#include "oaDesignDB.h"

// line_t: line segment in line-probing algorithm
class line_t : public std::pair<oa::oaPoint, oa::oaPoint> {
public:
    line_t() : std::pair<oa::oaPoint, oa::oaPoint>() {}
    line_t(const oa::oaPoint &lhs, const oa::oaPoint &rhs)
        : std::pair<oa::oaPoint, oa::oaPoint>(lhs, rhs) {}
    oa::oaBoolean contains(const oa::oaPoint &point);
};

#endif
