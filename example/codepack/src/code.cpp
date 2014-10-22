#include "ros/ros.h"
#include <libfunk/lib.hpp>
#include <libfunk/tt.hpp>

int main(int argc, char* argv[]){
  Library::libfunc();
  std::cout << Library::intfunc() << std::endl;
  Library::testfunc();
  tt::print();
}
