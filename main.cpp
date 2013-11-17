#include <iostream>
#include <fstream>
#include "oaDesignDB.h"
#include "route_core.h"
#include "NetSet.h"
#include "DRC.h"

using namespace std;
using namespace oa;

int main(int argc, char *argv[])
{
    if (argc != 5) {
        cerr << "Usage: ./main input_cell output_cell Connection_file";
        cerr << " Design rule file." << endl;
        return 1;
    }
    cout << "Routing Cell: " << argv[1] << endl;
    cout << "Output Cell: " << argv[2] << endl;
    cout << "Connection file: " << argv[3] << endl;
    cout << "Design rule file: " << argv[4] << endl;
    try {
        oaDesignInit(oacAPIMajorRevNumber, oacAPIMinorRevNumber, 3);

        oaNativeNS oaNs;
        oaString libraryPath("./DesignLib");
        oaString library("DesignLib");
        oaString top_cell(argv[1]);
        oaString new_top_cell(argv[2]);
        oaString layout_view("layout");
        oaScalarName libraryName(oaNs, library);
        oaScalarName cellName(oaNs, top_cell);
        oaScalarName newCellName(oaNs, new_top_cell);
        oaScalarName layoutView(oaNs, layout_view);

        // open the libs defined in "lib.def"
        oaLibDefList::openLibs();

        // locate the library
        oaLib *lib = oaLib::find(libraryName);

        if (!lib) {
            if (oaLib::exists(libraryPath)) {
                lib = oaLib::open(libraryName, libraryPath);
            }
            else {
                lib = oaLib::create(libraryName, libraryPath);
            }
            if (lib) {
                // update the lib def list
                oaLibDefList *list = oaLibDefList::getTopList();
                if (list) {
                    oaString topListPath;
                    list->getPath(topListPath);
                    list->get(topListPath, 'a');
                    oaLibDef *newLibDef = oaLibDef::create(list, libraryName, libraryPath);
                    list->save();
                }
            }
            else {
                cerr << "Error: Unable to create " << libraryPath << "/";
                cerr << library << endl;
                return 1;
            }
        }

        // open the design now
        oaDesign *design = oaDesign::open(libraryName, cellName, layoutView, 'r');

        design->saveAs(libraryName, newCellName, layoutView);
        oaScalarName name_buffer;
        oaString string_buffer;
        design->getLibName(name_buffer);
        name_buffer.get(oaNs,string_buffer);
        cout << "The library name for this design is : " << string_buffer << endl;

        design->getCellName(name_buffer);
        name_buffer.get(oaNs,string_buffer);
        cout << "The cell name for this design is : " << string_buffer << endl;

        design->getViewName(name_buffer);
        name_buffer.get(oaNs,string_buffer);
        cout << "The view name for this design is : " << string_buffer << endl;

        // open oaTech
        oaTech *tech = oaTech::open(lib, 'a');

        // read connection file and design rule file
        ifstream file;

        file.open(argv[3]);
        if (!file.good()) {
            cerr << "Cannot open file: " << argv[3] << endl;
            exit(1);
        }
        NetSet_t net(file);
        file.close();
        file.open(argv[4]);
        if (!file.good()) {
            cerr << "Cannot open file: " << argv[4] << endl;
            exit(1);
        }
        DRC_t designRule(file);
        file.close();

        // start routing
        route(design, tech, net, designRule);

        // save the design
        design->saveAs(libraryName, newCellName, layoutView);
        // save the tech with new created layers
        tech->save();
        design->close();
    }
    catch (oaException &excp) {
        cout << "ERROR: " << excp.getMsg() << endl;
        exit(1);
    }
    return 0;
}
