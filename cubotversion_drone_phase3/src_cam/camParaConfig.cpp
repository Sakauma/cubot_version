#include "daHua_usb/camParaConfig.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

void getCamParaFromYml(const std::string &filename, daHuaPara_str &_daHuaPara)
{
    FileStorage fs2(filename, FileStorage::READ);
    if (fs2.isOpened() != true)
    {
        _daHuaPara.IsReadYmlSucc = false;
        std::cerr<<"Failed to get the configur file "<<std::endl;
        return ;
    }

    /*string ime_srcPath = "/home/snow/RoboMaster/";
    fs << "ime_srcPath" << ime_srcPath;
    string img_savePath = "/home/snow/RoboMaster/";
    fs << "img_savePath" << img_savePath;

    int start_num = 1;
    fs << "start_num" << start_num;
    int end_num = 100;
    fs << "end_num" << end_num;

    int color = 1;
    fs << "color" << color;

    int bin_thresh = 90;
    fs << "bin_thresh" << bin_thresh;
    int lgts_bin_thresh = 180;
    fs << "lgts_bin_thresh" << lgts_bin_thresh;

    int lgt_min_area = 1000;
    fs << "lgt_min_area" << lgt_min_area;
    int lgt_max_area = 15;
    fs << "lgt_max_area" << lgt_max_area;*/

    _daHuaPara.IsGrabModeContinuesMy = (int)fs2["IsGrabModeContinuesMy"];
    _daHuaPara.IsAutoExposureMy = (int)fs2["IsAutoExposureMy"];
    _daHuaPara.exposeTimeMy = (int)fs2["exposeTimeMy"];
    _daHuaPara.gainRawMy    = (double)fs2["gainRawMy"];
    _daHuaPara.GammaMy      = (double)fs2["GammaMy"];
    _daHuaPara.frameRateMy  = (int)fs2["frameRateMy"];


    _daHuaPara.redBalanceRatioMy   = (double)fs2["redBalanceRatioMy"];
    _daHuaPara.greenBalanceRatioMy = (double)fs2["greenBalanceRatioMy"];
    _daHuaPara.blueBalanceRatioMy  = (double)fs2["blueBalanceRatioMy"];

    _daHuaPara.imgWidth = (int)fs2["imgWidth"];
    _daHuaPara.imgHeight = (int)fs2["imgHeight"];

    _daHuaPara.x_offset = (int)fs2["x_off"];
    _daHuaPara.y_offset = (int)fs2["y_off"];


    _daHuaPara.IsReadYmlSucc = true;

    fs2.release();
}

void camParaConfig(ICameraPtr &cameraSptr, const daHuaPara_str &_daHuaPara, int &IsSetCamParaSucc)
{
    int success_flag = 0;
    success_flag += setGainRaw(cameraSptr, _daHuaPara.gainRawMy);
    success_flag += setGamma(cameraSptr, _daHuaPara.GammaMy);

    success_flag += setGrabMode(cameraSptr, _daHuaPara.IsGrabModeContinuesMy);
    success_flag += setExposureTime(cameraSptr, _daHuaPara.exposeTimeMy, _daHuaPara.IsAutoExposureMy);

    success_flag += setAcquisitionFrameRate(cameraSptr, _daHuaPara.frameRateMy);
    success_flag += setBalanceRatio(cameraSptr, _daHuaPara.redBalanceRatioMy,
                                    _daHuaPara.greenBalanceRatioMy, _daHuaPara.blueBalanceRatioMy);

    success_flag += setROI(cameraSptr, _daHuaPara.x_offset, _daHuaPara.y_offset,
                           _daHuaPara.imgWidth, _daHuaPara.imgHeight);
    success_flag += setResolution(cameraSptr, _daHuaPara.imgWidth, _daHuaPara.imgHeight);

    IsSetCamParaSucc = success_flag;
}
