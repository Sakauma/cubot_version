//
// Created by jason on 18-6-2.
//

#ifndef TEST_OPENCV_PRO_FILE_H
#define TEST_OPENCV_PRO_FILE_H


#include <iostream>


#include <cstring>
#include <stdio.h>
#include <dirent.h>
#include <regex>

void get_file_list(std::string img_root,std::vector<std::string> & name_list,std::string post_fix);

std::string int_2_strind(int num);
int string_2_int(std::string s);

#endif //TEST_OPENCV_PRO_FILE_H
