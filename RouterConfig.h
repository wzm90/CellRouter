#ifndef ROUTERCONFIG_H_
#define ROUTERCONFIG_H_

#include <utility>
#include <vector>
#include <map>
#include "oaDesignDB.h"
#include "Net.h"

// line_t: line segment in line-probing algorithm
typedef std::pair<oa::oaPoint, oa::oaPoint> line_t; 

// PointSet_t: containter for storing escape points, 
// used in line-probing algorithm
typedef std::vector<oa::oaPoint> PointSet_t;

// LineSet_t: containters for storing line segments, 
// used in line-probing algorithm
typedef std::map<oa::oaCoord, line_t> LineSet_t;

// Orient_t: type of orientation flag in line-probing algorithm
typedef enum {HORIZONTAL, VERTICAL, BOTH} Orient_t;

// Nets_t: containter for storing nets, Nets_t[0] stores VDD, 
// Nets_t[1] stores VSS, Nets_t[2] stores S, Nets_t[3] stores IO
typedef std::vector<std::vector<Net_t> > Nets_t;

typedef Nets_t::size_type NetType_t;

typedef Nets_t::value_type::const_iterator net_iterator;

#endif
