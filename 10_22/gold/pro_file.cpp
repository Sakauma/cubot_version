//
// Created by jason on 18-6-2.
//

#include "gold/pro_file.h"

int string_2_int(std::string s)
{
    int num;
    std::stringstream ss;
    ss<<s;
    ss>>num;
    return num;
}

std::string int_2_strind(int num)
{
    std::string s;
    std::stringstream ss;
    ss<<num;
    ss>>s;
    return s;
}


void get_file_list(std::string img_root,std::vector<std::string> & name_list,std::string post_fix)
{
    DIR * dir=opendir(img_root.c_str());
    std::vector<int> num_list;
    struct dirent *ptr;
    while((ptr=readdir(dir))!=NULL)
    {
        if(ptr->d_name[0] == '.')
            continue;
//        printf("%s \n",ptr->d_name);

        std::string name=ptr->d_name;
//        std::cout<<name<<std::endl;

//        std::cout<<name<<std::endl;
        name = name.substr(0, name.length()-4);
        num_list.push_back(string_2_int(name));

    }
    closedir(dir);

    sort(num_list.begin(), num_list.end(),std::less<int>());//升
    for(int i=0;i<num_list.size();i++)
    {
        name_list.push_back(img_root+int_2_strind(num_list[i])+post_fix);
        
//        std::cout<<"sort list: "<<name_num_list[i]<<std::endl;
//        std::cout<< img_root+int_2_strind(num_list[i])+post_fix <<std::endl;
    }

}




