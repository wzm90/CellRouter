#include <iostream>
#include <string>
#include "NetSet.h"

using namespace std;
using namespace oa;

// read from netlist.txt and store netlist
NetSet_t::NetSet_t(ifstream &file)
    :_allNets(4)
{
    file.clear();
    file.seekg(0);
    string line;
    do {
        getline(file, line);
        if (file.good()) {
#ifdef DEBUG    
            cout << line << endl;
#endif
            Net_t net;
            NetType_t type;
            type = parseNet(net, line);
            addNet(type, net);
        }
    } while (file.good());
}

// parse input text file and extract one net
NetType_t
NetSet_t::parseNet(Net_t &net, const string &line)
{
    istringstream ss(line);
    oaCoord coord1, coord2;
    while (ss >> coord1 >> coord2) {
#ifdef DEBUG
        cout << "Read contact positions: " << coord1 << " and " << coord2;
        cout << endl;
#endif
        oaPoint point(coord1, coord2);
        net.push_back(point);
    }
    string type;
    ss.clear();
    if (ss >> type) {
        if (type == "VDD") {
            return 0;
        } else if (type == "VSS") {
            return 1;
        } else if (type == "S") {
            return 2;
        } else if (type.find("IO/") != string::npos) {
            size_t pos = type.find("IO/") + 3;
            oaString portName(string(type, pos).c_str());
#ifdef DEBUG
            cout << "The port name is: " << portName << endl;         
#endif
            net.setPortName(portName);
            return 3;
        } else {
            cerr << "Unknown net type: " << type << endl;
            exit(1);
        }
    } else {
        cerr << "Invalid netlist format." << endl;
        exit(1);
    }
}
