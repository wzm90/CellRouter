#ifndef DRC_H_
#define DRC_H_
#include <fstream>
#include "oaDesignDB.h"

class DRC_t {
public:
    DRC_t(std::ifstream &file);
    oa::oaUInt4 metalWidth() const { return _metalWidth; }
    oa::oaUInt4 metalSpacing() const { return _metalSpacing; }
    oa::oaUInt4 viaExtension() const { return _viaExtension; }
    oa::oaUInt4 metalArea() const { return _metalArea; }
    oa::oaUInt4 viaWidth() const { return _viaWidth; }
    oa::oaUInt4 viaHeight() const { return _viaHeight; }
private:
    void setMetalWidth(oa::oaUInt4 width) { _metalWidth = width; }
    void setMetalSpacing(oa::oaUInt4 space) { _metalSpacing = space; }
    void setViaExtension(oa::oaUInt4 extension) { _viaExtension = extension; }
    void setMetalArea(oa::oaUInt4 area) { _metalArea = area; }
    void setViaWidth(oa::oaUInt4 vwidth) { _viaWidth = vwidth; }
    void setViaHeight(oa::oaUInt4 vheight) { _viaHeight = vheight; }
    // design rules (in coordinate units, not in nm)
    oa::oaUInt4 _metalWidth;
    oa::oaUInt4 _metalSpacing;
    oa::oaUInt4 _viaExtension;
    oa::oaUInt4 _metalArea;
    oa::oaUInt4 _viaWidth;
    oa::oaUInt4 _viaHeight;
};

#endif
