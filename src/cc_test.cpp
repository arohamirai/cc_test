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
    // 读写同一文件
    ifstream json_file_in("../file/info_WorldRefer.json", std::ios::binary);

    Json::CharReaderBuilder  builder;
    Json::StreamWriterBuilder writer;
    Json::Value root;

    if(!json_file_in.is_open())
    {
        cout<<"can't open json file."<<endl;
        return 0;
    }

    JSONCPP_STRING errs;

    if(Json::parseFromStream(builder, json_file_in, &root, &errs))
    {
        cout << "nodes:" << endl;
        for(unsigned int i = 0; i < root["nodes"].size(); i++)
        {
            // 修改json字段
            for(unsigned int j = 0; j < root["nodes"][i]["pose"].size(); j++)
            {
                root["nodes"][i]["pose"][j] = j;
                cout<<root["nodes"][i]["pose"][j]<<endl;
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
    else
    {
        cout<<"parseFromStream error: "<<errs<<endl;
    }

    json_file_in.close();

    ofstream json_file_out("../file/info_WorldRefer.json", std::ios::binary | std::ios::trunc);

    // 写json文件, writer 可以设置写的格式，比如字段建用空格还是换行符分隔
    string str_out = Json::writeString(writer, root);
    json_file_out << str_out;
    json_file_out.close();

    return 0;
}
