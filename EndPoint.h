// The class EndPoint_t is used for line-probing algorithm only.
#ifndef ENDPOINT_H_
#define ENDPOINT_H_

#include "oaDesignDB.h"
#include "RouterType.h"

class EndPoint_t {
public:
    EndPoint_t(oa::oaCoord x, oa::oaCoord y, oa::oaInt4 id);
    Orient_t orient() const { return _orient; }
    void setOrient(Orient_t newOrient) { _orient = newOrient; }

    PointSet_t &escapePoints() { return _escapePoints; }
    const PointSet_t &escapePoints() const { return _escapePoints; }
    oa::oaPoint &getObjectPoint() { return _escapePoints.back(); }
    const oa::oaPoint &getObjectPoint() const { return _escapePoints.back(); }
    void addEscapePoint(const oa::oaPoint &point) { _escapePoints.push_back(point); }

    // we get _cornerPoints only after an intersection point is found.
    void getCornerPoints(const oa::oaPoint &intersectionPoint);
    // cornerPoints() can only be called after an 
    // intersectionPoint is found!
    PointSet_t &cornerPoints() { return _cornerPoints; }

    //EndPoint_t::LineSet_t &hlines() { return _hlines; }
    void addHline(const line_t &newline);

    //EndPoint_t::LineSet_t &vlines() { return _vlines; }
    void addVline(const line_t &newline);

    bool noEscape() const { return _noEscape; }
    void setNoEscape(bool val) { _noEscape = val; }

    bool isIntersect(const line_t &line, oa::oaPoint &intersectionPoint) const;
    bool onEscapeLines(const oa::oaPoint &point, Orient_t orient) const;
    oa::oaInt4 netID() const { return _netID; }
private:
    typedef std::map<oa::oaCoord, line_t> LineSet_t;

    Orient_t _orient;
    PointSet_t _escapePoints;
    LineSet_t _hlines;
    LineSet_t _vlines; 
    bool _noEscape;
    oa::oaInt4 _netID;
    PointSet_t _cornerPoints;
};

#endif

