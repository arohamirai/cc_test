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

    virtual int print() = 0;

    T t;
};

template<class T>
BaseClass<T>::~BaseClass()
{

}

class DepriveClass:public BaseClass<int>
{
public:
    DepriveClass(int a){};
    ~DepriveClass(){};

    virtual int print()
    {
        std::cout<<"DepriveClass:"<<std::endl;
        return 0;
    }

};

class DepriveClass1:public BaseClass<std::string>
{
public:
    DepriveClass1(std::string s){};
    ~DepriveClass1(){};

    virtual int print()
    {
        std::cout<<"DepriveClass1:"<<std::endl;
        return 1;
    }

};

class my_visitor : public boost::static_visitor<int>
{
public:
    template<typename T>
    int operator()(T x) const
    {
        return x->print();
    }
};


int main(int argc, char **argv)
{
    std::shared_ptr<DepriveClass> a = std::make_shared<DepriveClass>(1);
    std::shared_ptr<DepriveClass1> b = std::make_shared<DepriveClass1>("s");
    boost::variant< std::shared_ptr<DepriveClass>, std::shared_ptr<DepriveClass1>> u;

    int result = 0;

    u = a;
    result = boost::apply_visitor( my_visitor(), u );
    std::cout<<"result:"<<result<<std::endl;

    u = b;
    result = boost::apply_visitor( my_visitor(), u );
    std::cout<<"result:"<<result<<std::endl;

    return 0;
}
