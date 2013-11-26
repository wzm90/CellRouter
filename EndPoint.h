// The class EndPoint_t is used for line-probing algorithm only.
#ifndef ENDPOINT_H_
#define ENDPOINT_H_

#include "oaDesignDB.h"
#include "RouterConfig.h"

class EndPoint_t {
public:
    EndPoint_t(oa::oaCoord x=0, oa::oaCoord y=0);
    Orient_t orient() const { return _orient; }
    void setOrient(Orient_t newOrient) { _orient = newOrient; }

    PointSet_t &escapePoints() { return _escapePoints; }
    const PointSet_t &escapePoints() const { return _escapePoints; }
    oa::oaPoint &getObjectPoint() { return _escapePoints.back(); }
    const oa::oaPoint &getObjectPoint() const { return _escapePoints.back(); }
    void addEscapePoint(const oa::oaPoint &point) { _escapePoints.push_back(point); }

    LineSet_t &hlines() { return _hlines; }
    void addHline(const line_t &newline);

    LineSet_t &vlines() { return _vlines; }
    void addVline(const line_t &newline);

    bool noEscape() const { return _noEscape; }
    void setNoEscape(bool val) { _noEscape = val; }

    bool isIntersect(const line_t &line) const;
    bool onEscapeLines(const oa::oaPoint &point, Orient_t orient) const;
    bool justStart() const { return (_escapePoints.size() == 1); }
private:
    Orient_t _orient;
    PointSet_t _escapePoints;
    LineSet_t _hlines;
    LineSet_t _vlines; 
    bool _noEscape;
};

#endif

