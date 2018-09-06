#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
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

    std::shared_ptr<int> x = std::make_shared<int>(4);

    std::shared_ptr<DepriveClass> a = std::make_shared<DepriveClass>();
    std::shared_ptr<DepriveClass1> b = std::make_shared<DepriveClass1>();
    boost::variant< std::shared_ptr<DepriveClass>, std::shared_ptr<DepriveClass1>> u;

    u = a;
    boost::get<std::shared_ptr<DepriveClass>>(u)->print(10);

    u = b;
    boost::get<std::shared_ptr<DepriveClass1>>(u)->print("hello world");



    return 0;
}
