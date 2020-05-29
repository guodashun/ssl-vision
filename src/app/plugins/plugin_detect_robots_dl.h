#ifndef PLUGIN_DETECT_ROBOTS_DL_H
#define PLUGIN_DETECT_ROBOTS_DL_H
#include <visionplugin.h>
#include <torch/torch.h>
#include <torch/script.h>
#include "field_filter.h"
#include "cmvision_histogram.h"
#include "cmpattern_teamdetector.h"
#include "cmpattern_team.h"
#include "vis_util.h"
#include "lut3d.h"
#include "VarNotifier.h"

typedef torch::jit::script::Module Dict;

class PluginDetectRobotsDL : public VisionPlugin
{
public:
    PluginDetectRobotsDL(FrameBuffer * _buffer);
    virtual ProcessResult process(FrameData * data, RenderOptions * options);
private:
    Dict location_model, team_model, id_model, orientation_model;
    bool isRecord = false;
    CMVision::RegionTree reg_tree;
    void buildRegionTree(CMVision::ColorRegionList * colorlist);
};

#endif // PLUGIN_DETECT_ROBOTS_DL_H
