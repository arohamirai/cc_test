#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <jsoncpp/json/json.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/dir.h>
#include <sys/errno.h>
#include <time.h>

using namespace std;

bool gotRecordPath(string &record_path, int &record_index)
{
  string record_name = "templated_laser_localization_record";
  string record_filename_suffix = "_localization";
  record_path = "";
  record_index = 0;
  DIR* dir;
  // check parent folder
  string parent_path =  getenv("HOME") +string("/") + record_name;
  dir = opendir(parent_path.c_str());
  if(!dir)
  {
    cout<<parent_path<< " dir doesn't exist, start to create it.... : "<<endl;
    if(mkdir(parent_path.c_str(),S_IRWXU|S_IRWXG|S_IRWXO))
    {
      cout<< "parent folder: " << parent_path <<" dir create failed, error ID: " <<errno << ", description: " <<strerror(errno)<<endl;
      return false;
    }
    cout<< "parent folder: "<< parent_path << " dir create succeed." <<endl;
  }
  else
  {
      cout<< "parent folder: "<<parent_path<<" dir already exist, mkdir skiped."<<endl;
    closedir(dir);
    dir = NULL;
  }

  // create totay folder
  struct tm* ptm;
  long ts;
  int y,m,d;

  ts = time(NULL);
  ptm = localtime(&ts);
  y = ptm-> tm_year+1900;
  m = ptm-> tm_mon+1;
  d = ptm-> tm_mday;

  string today_folder = std::to_string(y) +std::to_string(m)+std::to_string(d);
  record_path =parent_path+"/" +today_folder;
  dir = opendir(record_path.c_str());
  if(!dir)
  {
    if(mkdir(record_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO))
    {
      cout << "record dir: "<< record_path << " create failed, error description: " << strerror(errno) <<endl;
      return false;
    }
  }
  else
  {
      cout << "record dir: "<< record_path << " already exist." <<endl;
    // find the index
    struct dirent *d_ent;
    while((d_ent = readdir(dir)) != 0)
    {
        string filename = string(d_ent->d_name);
        int npos = 0;
        npos = filename.find(record_filename_suffix);
        if(npos != string::npos)
            record_index++;
    }
    closedir(dir);
    dir = NULL;
  }

  // only keep 10 folders, include totay's folder
  int record_days = 10;
  int num_folders = 0;
  struct dirent **namelist;
  num_folders = scandir(parent_path.c_str(),&namelist,0,alphasort);
  if(num_folders == -1)
  {
      cout << "scan files or folders in parent folder: "<< parent_path << "  failed." <<endl;
    return false;
  }
  num_folders -= 2;
  //cout<<"num_folders:"<<num_folders<<endl;

  struct dirent **folder_list = namelist;
  int num_folders_remind = num_folders;
  while(num_folders_remind > record_days){
    if(strcmp((*folder_list)->d_name, ".") == 0 ||
       strcmp((*folder_list)->d_name, "..") == 0 ||
       strcmp((*folder_list)->d_name, today_folder.c_str()) == 0)
    {
      folder_list++;
      continue;
    }

    string absolute_folder_name = parent_path + "/" + (*folder_list)->d_name;
    DIR* dir = opendir(absolute_folder_name.c_str());
    // iterator files
    struct dirent* d_ent;
    while((d_ent = readdir(dir)) != 0)
    {
      if(strcmp((*folder_list)->d_name, ".") != 0 &&
         strcmp((*folder_list)->d_name, "..") != 0)
      {
        string file_path = absolute_folder_name + "/" + string(d_ent->d_name);

        std::remove(file_path.c_str());

      }
    }
    if(rmdir(absolute_folder_name.c_str()))
    {
        cout << "delete folder: "<< absolute_folder_name << " failed, failed description: " <<strerror(errno)<<endl;
      return false;
    }
    folder_list++;
    num_folders_remind--;
  }

  for(int i = 0; i < num_folders + 2; i++)
    free(namelist[i]);
  free(namelist);

  return true;
}



int main(int argc, char **argv)
{
    string record_path;
    int record_index;

    bool succeed = gotRecordPath(record_path, record_index);

    if(succeed)
    {
        cout<<"record_path: "<< record_path << "     record_index: "<< record_index<<endl;
    }
    else
    {
        cout << " failed."<<endl;
    }
    return 0;
}
