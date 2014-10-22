#include "ros/ros.h"
//#include <libfunk/lib.hpp>
//#include <libfunk/tt.hpp>
#include <example/libpack/src/tt.hpp>
#include <example/libpack/src/lib.hpp>

int main(int argc, char* argv[]){
  Library::libfunc();
  std::cout << Library::intfunc() << std::endl;
  Library::testfunc();
  tt::print();
  //tt::print();
}
