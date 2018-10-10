#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <jsoncpp/json/json.h>
using namespace std;


int main(int argc, char **argv)
{
    ifstream json_file("../file/info_WorldRefer.json", ios::binary);
    if(!json_file.is_open())
    {
        cout<<"can't open json file."<<endl;
        return 0;
    }

    Json::Reader reader;
    Json::Value root;
    if(reader.parse(json_file, root))
    {
        cout<<"dictCategory: "<< root["dictCategory"]["0"]<<endl;
        cout<<"dictEvent: "<< root["dictEvent"]["0"]<< "  "<< root["dictEvent"]["1"]<<endl;

        //读取数组信息
        cout << "directActions:" << endl;
        for(unsigned int i = 0; i < root["directActions"].size(); i++)
        {
            string ach = root["directActions"][i].asString();
            cout << ach << endl;
        }
    }



    return 0;
}
