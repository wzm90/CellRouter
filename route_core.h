#ifndef ROUTE_CORE_H_
#define ROUTE_CORE_H_

#include "oaDesignDB.h"
#include "Netlist.h"
#include "DRC.h"

void route(oa::oaDesign *design, oa::oaTech *tech, const Netlist_t &netlist,\
           const DRC_t &designRule);

#endif
