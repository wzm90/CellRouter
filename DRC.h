#ifndef DRC_H_
#define DRC_H_
#include <fstream>
#include "oaDesignDB.h"

class DRC_t {
public:
    DRC_t(std::ifstream &file);
    oa::oaInt4 metalWidth() const { return _metalWidth; }
    oa::oaInt4 metalSpacing() const { return _metalSpacing; }
    oa::oaInt4 viaExtension() const { return _viaExtension; }
    oa::oaInt4 metalArea() const { return _metalArea; }
    oa::oaInt4 viaWidth() const { return _viaWidth; }
    oa::oaInt4 viaHeight() const { return _viaHeight; }
private:
    void setMetalWidth(oa::oaInt4 width) { _metalWidth = 10 * width; }
    void setMetalSpacing(oa::oaInt4 space) { _metalSpacing = 10 * space; }
    void setViaExtension(oa::oaInt4 extension) { _viaExtension = 10 * extension; }
    void setMetalArea(oa::oaInt4 area) { _metalArea = 100 * area; }
    void setViaWidth(oa::oaInt4 vwidth) { _viaWidth = 10 * vwidth; }
    void setViaHeight(oa::oaInt4 vheight) { _viaHeight = 10 * vheight; }
    // design rules (in coordinate units, not in nm)
    oa::oaInt4 _metalWidth;
    oa::oaInt4 _metalSpacing;
    oa::oaInt4 _viaExtension;
    oa::oaInt4 _metalArea;
    oa::oaInt4 _viaWidth;
    oa::oaInt4 _viaHeight;
};

#endif
