#ifndef ROUTERCONFIG_H_
#define ROUTERCONFIG_H_

#include <utility>
#include <vector>
#include <map>
#include "oaDesignDB.h"

// line_t: line segment in line-probing algorithm
typedef std::pair<oa::oaPoint, oa::oaPoint> line_t; 

// PointSet_t: containter for storing escape points, 
// used in line-probing algorithm
typedef std::vector<oa::oaPoint> PointSet_t;

//typedef std::map<oa::oaCoord, line_t> LineSet_t;

// Orient_t: type of orientation flag in line-probing algorithm
typedef enum {HORIZONTAL, VERTICAL, BOTH} Orient_t;

typedef enum {VDD, VSS, S, IO} NetType_t;

#endif
