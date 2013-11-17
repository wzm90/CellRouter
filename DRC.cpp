#include <iostream>
#include <string>
#include "DRC.h"

using namespace std;
using namespace oa;

DRC_t::DRC_t(ifstream &file)
{
    file.clear();
    file.seekg(0);
    string line;
    getline(file, line);
    if (file.good()) {
#ifdef DEBUG
        cout << line << endl;
#endif
        istringstream ss(line);
        oaUInt4 value;
        int i;
        for (i = 0; i < 6 && (ss >> value); ++i) {
            switch (i) {
            case 0:
                setMetalWidth(value);
                break;
            case 1:
                setMetalSpacing(value);
                break;
            case 2:
                setViaExtension(value);
                break;
            case 3:
                setMetalArea(value);
                break;
            case 4:
                setViaWidth(value);
                break;
            case 5:
                setViaHeight(value);
                break;
            default:
                cerr << "Error in switch..." << endl;
                exit(1);
            } 
        }
        if (i != 6) {
            cerr << "Invalid design rule format: " << line << endl;
            exit(1);
        }
    } else {
        cerr << "Cannot read design rule file." << endl;
        exit(1);
    }
}
