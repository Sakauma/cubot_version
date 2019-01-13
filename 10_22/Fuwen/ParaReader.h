//
// Created by abds on 18-5-30.
//

#ifndef ROBOMASTER_PARAREADER_H
#define ROBOMASTER_PARAREADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>

#include "opencv2/opencv.hpp"

class ParaReader
{
public:
    ParaReader();

    ParaReader(std::string filename)
    {
        std::ifstream fin( filename.c_str() );

        if (!fin)
        {
            std::cerr<<"parameter file does not exist."<<std::endl;
            return;
        }
        std::string key;
        std::vector<cv::Point2f> points;
        while(!fin.eof())
        {
            std::string str;
            std::getline( fin, str);
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }
            std::istringstream iss(str);
            std::string value;
            while(iss>>value){


                int pose = value.find(',');
                if(pose == -1){
                    key = value;
                }
                else{
                    float x = atof(value.substr(0,pose).c_str());

                    float y = atof(value.substr(pose+1,str.length()).c_str());
                    cv::Point2f point(x,y);
                    points.push_back(point);
                }

            }





            if(points.size() == 4){

                map_list[key] = points;
                points.clear();
            }


            if ( !fin.good() )
                break;
        }
    }
    bool getData( std::string key ,std::vector<cv::Point2f> &points)
    {
        std::map<std::string, std::vector<cv::Point2f>>::iterator iter = map_list.find(key);
        if (iter == map_list.end())
        {
            std::cerr<<"Parameter name "<<key<<" not found!"<<std::endl;
            return false;
        }
        points.clear();
        points = iter->second;
        return true;
    }
public:
    std::map<std::string,std::vector<cv::Point2f>> map_list;
};


#endif //ROBOMASTER_PARAREADER_H
