#include "plugin_detect_robots_dl.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <QDebug>

PluginDetectRobotsDL::PluginDetectRobotsDL(FrameBuffer * _buffer)
    : VisionPlugin (_buffer)
{
    location_model = torch::jit::load("./model/local.pth");
    team_model = torch::jit::load("./model/team.pth");
    id_model = torch::jit::load("./model/id.pth");
    orientation_model = torch::jit::load("./model/orien.pth");
    isRecord = true;
}

void PluginDetectRobotsDL::buildRegionTree(CMVision::ColorRegionList * colorlist) {
  reg_tree.clear();
  int num_colors=colorlist->getNumColorRegions();
  for(int c=0;c<num_colors;c++) {
    //ONLY ADD ROBOT MARKER COLORS:
    if (c!= color_id_clear && c!=color_id_field && c!= color_id_ball && c!= color_id_black) {
      CMVision::Region *reg = colorlist->getRegionList(c).getInitialElement();
      while(reg!=0) {
        reg_tree.add(reg);
        reg = reg->next;
      }
    }
  }
  reg_tree.build();
}

ProcessResult PluginDetectRobotsDL::process(FrameData * data, RenderOptions * options) {
    cv::Mat raw_image = data->map.get("cv_pic");
    raw_image.resize(1500, 1000);
    torch::Tensor raw_image_tensor = torch::from_blob(raw_image.data, {1, raw_image.rows, raw_image.cols, 3}, torch::kByte);
    raw_image_tensor = raw_img_tensor.permute({0, 3, 1, 2});
    raw_img_tensor = raw_img_tensor.toType(torch::kFloat);
    raw_img_tensor = raw_img_tensor.div(255);
    torch::Tensor local_detec = location_model.forward({raw_img_tensor}).toTensor();
    for (auto i : local_detec.size()) {
        int j = 0;
        while(local_detec[0, i, j, 0] >= 0.2) {
            double score = detections[0, i, j, 0].item();
            cv::Mat robot_img = img[(int(pt[1]) - 50):(int(pt[3]) + 50),
                                (int(pt[0]) - 50):(int(pt[2]) + 50)];
            torch::Tensor robot_tensor = torch::from_blob(robot_img.data, {1, robot_img.rows, robot_img.cols, 3}, torch::kByte);
            torch::Tensor team = team_model.forward({robot_tensor}).toTensor().max(1, true);
            torch::Tensor id = id_model.forward({robot_tensor}).toTensor().max(1, true);
            torch::Tensor orien = orientation_model.forward({robot_tensor}).toTensor();
            cv2::rectangle(raw_image,
                              (int(pt[0]), int(pt[1])),
                              (int(pt[2]), int(pt[3])),
                              COLORS[i % 3], 2);
            cv2::putText(raw_image, "team: " + str(team.item()), (int(pt[0]), int(pt[1]) - 40), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2);
            cv2::putText(raw_image, "numb: " + str(robot_id.item()), (int(pt[0]), int(pt[1]) - 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2);
            cv2::putText(raw_image, "orie: " + str(orientation), (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2);
            j++;
        }
    }
    if (isRecord) {
        cv2::imwrite(pic_dir + "/test_out/pic_" + str(img_i) + '.png', raw_image);
    }
    CMPattern::TeamDetector * detector;
    detector->update(id, team, orien, colorlist, reg_tree);
    return ProcessingOk;

    // qDebug() << "fuck run!";
}
