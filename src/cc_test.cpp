#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <jsoncpp/json/json.h>
using namespace std;

// jsoncpp  reference : https://www.cnblogs.com/DswCnblog/p/6678708.html

int main(int argc, char **argv)
{
    ifstream json_file_in("../file/info_WorldRefer.json", ios::binary);
    ofstream json_file_out("../file/info_WorldRefer_out.json", ios::binary);

    Json::Reader reader(Json::Features::strictMode());
    Json::StyledWriter writer;
    Json::Value root;


    if(!json_file_in.is_open())
    {
        cout<<"can't open json file."<<endl;
        return 0;
    }

    if(reader.parse(json_file_in, root))
    {
        cout << "nodes:" << endl;
        for(unsigned int i = 0; i < root["nodes"].size(); i++)
        {
            // 修改json字段
            for(unsigned int j = 0; j < root["nodes"][i]["pose"].size(); j++)
            {
                root["nodes"][i]["pose"][j] = 0;
            }

            // 读json字段
            cout << "worldPose: "
                 <<root["nodes"][i]["worldPose"]["position"]["x"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["position"]["x"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["position"]["y"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["position"]["z"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["orientation"]["x"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["orientation"]["y"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["orientation"]["z"].asFloat()<<"  "
                 <<root["nodes"][i]["worldPose"]["orientation"]["w"].asFloat()<<"  ";
            cout << endl;
            cout<<"==========================="<<endl;
        }
    }

    // 写json文件
    string str_out = writer.write(root);
    json_file_out << str_out;

    json_file_in.close();
    json_file_out.close();

    return 0;
}
