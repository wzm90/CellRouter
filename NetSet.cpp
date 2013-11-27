#include <iostream>
#include <string>
#include "NetSet.h"

using namespace std;
using namespace oa;

// read from netlist.txt and store netlist
NetSet_t::NetSet_t(ifstream &file)
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
            parseAddNet(line);
        }
    } while (file.good());
#ifdef DEBUG 
    const_iterator netIter;
    for (netIter = this->begin(); netIter != this->end(); ++netIter)
    {
        // print each net
        cout << "ID: " << netIter->id() << " port: " << netIter->portName();
        cout << " type: ";
        if (netIter->type() == VDD) cout << "VDD";
        else if (netIter->type() == VSS) cout << "VSS";
        else if (netIter->type() == S) cout << "S";
        else if (netIter->type() == IO) cout << "IO";
        else cout << "NULL";
        cout << endl;
        Net_t::const_iterator it;
        for (it = netIter->begin(); it != netIter->end(); ++it) {
            cout << "(" << it->x() << ", " << it->y() << ") ";
        }
        cout << endl;
    }  
#endif
}

// parse input text file and extract one net
void
NetSet_t::parseAddNet(const string &line)
{
    istringstream ss(line);
    oaCoord coord1, coord2;
    vector<oaPoint> points;
    NetType_t type;
    oaString portName;

    while (ss >> coord1 >> coord2) {
#ifdef DEBUG
        cout << "Read contact positions: " << coord1 << " and " << coord2;
        cout << endl;
#endif
        points.push_back(oaPoint(coord1, coord2));
    }

    string typeName;
    ss.clear();
    if (ss >> typeName) {
        if (typeName == "VDD") {
            type = VDD;
        } 
        else if (typeName == "VSS") {
            type = VSS;
        }
        else if (typeName == "S") {
            type = S;
        }
        else if (typeName.find("IO/") != string::npos) {
            type = IO;
            size_t pos = typeName.find("IO/") + 3;
            portName = oaString(string(typeName, pos).c_str());
#ifdef DEBUG
            cout << "The port name is: " << portName << endl;         
#endif
        } else {
            cerr << "Unknown net type: " << typeName << endl;
            exit(1);
        }
        // push Net_t into NetSet_t 
        this->push_back(Net_t(points, this->size(), type, portName));
    } else {
        cerr << "Invalid netlist format." << endl;
        exit(1);
    }
}
