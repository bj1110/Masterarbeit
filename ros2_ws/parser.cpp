#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cstdlib>
#include <fstream>
#include <sstream>

int main(int argc, char* argv[]){

    if(argc<2){
        std::printf("No File has been given. Please provide location of file.\n");
        return 1;
    }
    std::string filename = argv[1];
    std::ifstream file{filename};
    std::string line;

    std::printf("Printing number of elements per line:\n");

    int len=0, row=0;
    while(std::getline(file, line)){
        ++row; 
        std::cout<< "\t"<< row << ": ";
        std::stringstream ss {line};   
        float a; 
        while(ss >> a){
            ++len; 
        }
        std::cout << len << std::endl; 
        len=0; 
    }

    return 0; 
}