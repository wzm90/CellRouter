#ifndef ROUTE_CORE_H_
#define ROUTE_CORE_H_

#include "oaDesignDB.h"
#include "NetSet.h"
#include "DRC.h"

void route(oa::oaDesign *design, oa::oaTech *tech, const NetSet_t &net,\
           const DRC_t &designRule);

#endif
