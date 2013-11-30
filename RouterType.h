#ifndef ROUTERTYPE_H_
#define ROUTERTYPE_H_

#include <utility>
#include <vector>
#include <map>
#include "oaDesignDB.h"
#include "line.h"


// PointSet_t: containter for storing escape points, 
// used in line-probing algorithm
typedef std::vector<oa::oaPoint> PointSet_t;

// Orient_t: type of orientation flag in line-probing algorithm
typedef enum {HORIZONTAL, VERTICAL, BOTH} Orient_t;

typedef enum {VDD, VSS, S, IO} NetType_t;

#endif
