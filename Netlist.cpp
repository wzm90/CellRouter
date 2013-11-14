#include <iostream>
#include <string>
#include "Netlist.h"

using std::string;
using std::getline;
using std::ifstream;
using std::cout;
using std::endl;

Netlist_t::Netlist_t(ifstream &file) {
    file.clear();
    file.seekg(0);
#ifdef DEBUG
    string line;
    do {
        getline(file, line);
        if (file.good()) {
            cout << line << endl;
        }
    } while (file.good());
#endif
}
