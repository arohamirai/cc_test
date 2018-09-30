#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
#include <iostream>
#include <string>
using namespace std;

int fun(void)
{
    int * crash = NULL;
    *crash = 1;

    return 1;
}

int main(int argc, char **argv)
{
    // trigger core dump
    int ret = fun();
    cout<<"ret:"<<ret<<endl;

    return 0;
}
