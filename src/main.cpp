// demo the Mat reading capabilities
#include <string>
#include "MatlabIO.hpp"
int main(int argc, char **argv) {

    // get the Matlab .Mat file from the command line
    string filename(argv[1]);

    // create a new reader
    MatlabIO matio;
    matio.open(filename, "-r");
    matio.close();
    return 0;
}
