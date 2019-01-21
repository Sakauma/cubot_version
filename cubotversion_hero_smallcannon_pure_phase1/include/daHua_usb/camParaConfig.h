#ifndef _CAMPARACONFIG_H_
#define _CAMPARACONFIG_H_

#include <iostream>
#include "daHuaCam.hpp"

typedef struct daHuaPara_str
{
    int IsGrabModeContinuesMy = 1;
    int IsAutoExposureMy = 0;
    int frameRateMy = 180;

    int imgWidth  = 1280;
    int imgHeight = 1024;
    int x_offset  = 0;
    int y_offset  = 0;

    double redBalanceRatioMy   = 1;
    double greenBalanceRatioMy = 1;
    double blueBalanceRatioMy  = 1;

    //=========Armor=========
    double GammaMy_Armor   = 0.7;
    double gainRawMy_Armor = 6.;
    int exposeTimeMy_Armor = 500;

    bool IsReadYmlSucc = false;

}daHuaPara_str;


void getCamParaFromYml(const std::string &filename, daHuaPara_str &_daHuaPara);
void camParaConfig(ICameraPtr &cameraSptr, const daHuaPara_str &_daHuaPara, int &IsSetCamParaSucc);
#endif
