#include "boost/variant.hpp"
#include <iostream>
#include <string>

template<class T>
class BaseClass
{
public:
    BaseClass(){};
    virtual ~BaseClass() = 0;

    virtual void print(T x) = 0;
};

template<class T>
BaseClass<T>::~BaseClass()
{

}

class DepriveClass:public BaseClass<int>
{
public:
    DepriveClass(){};
    ~DepriveClass(){};

    virtual void print(int x)
    {
        std::cout<<"DepriveClass:"<<x<<std::endl;
    }

};

class DepriveClass1:public BaseClass<std::string>
{
public:
    DepriveClass1(){};
    ~DepriveClass1(){};

    virtual void print(std::string x)
    {
        std::cout<<"DepriveClass1:"<<x<<std::endl;
    }

};






int main(int argc, char **argv)
{
//    boost::variant< int, std::string > u("hello world");
//    std::cout << u<<std::endl; // output: hello world

//    int result = boost::apply_visitor( my_visitor(), u );
//    std::cout << result << std::endl; // output: 11 (i.e., length of "hello world")

    DepriveClass a ;
    DepriveClass1 b;
    boost::variant< DepriveClass, DepriveClass1> u;

    u = a;
    boost::get<DepriveClass>(u).print(10);

    u = b;
    boost::get<DepriveClass1>(u).print("hello world");



    return 0;
}
