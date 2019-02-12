#ifndef _CAMPARACONFIG_H_
#define _CAMPARACONFIG_H_

#include <iostream>
#include "daHuaCam.hpp"

typedef struct daHuaPara_str
{
    int IsGrabModeContinuesMy = 1;
    int IsAutoExposureMy = 0;
    int exposeTimeMy     = 4000;


    int frameRateMy = 180;

    int imgWidth  = 1280;
    int imgHeight = 1024;
    int x_offset  = 0;
    int y_offset  = 0;

    double GammaMy   = 0.54;
    double gainRawMy = 5.;
    double redBalanceRatioMy   = 1;
    double greenBalanceRatioMy = 1;
    double blueBalanceRatioMy  = 1;

    bool IsReadYmlSucc = false;

}daHuaPara_str;

void getCamParaFromYml(const std::string &filename, daHuaPara_str &_daHuaPara);
void camParaConfig(ICameraPtr &cameraSptr, const daHuaPara_str &_daHuaPara, int &IsSetCamParaSucc);

#endif
