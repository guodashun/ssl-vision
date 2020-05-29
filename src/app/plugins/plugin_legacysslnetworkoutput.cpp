//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    plugin_legacysslnetworkoutput.cpp
  \brief   C++ Implementation: plugin_legacysslnetworkoutput
  \author  Joydeep Biswas, 2014
*/
//========================================================================
#include "plugin_legacysslnetworkoutput.h"
#include "plugin_visualize.h"
#include "image.h"
#include "tinyxml2.h"
#include <QDebug>
#include "time.h"
#include <QDateTime>

using namespace tinyxml2;

namespace  {
    const double ROBOT_PIXEL_SIZE = 18;
    const double BALL_PIXEL_SIZE = 5;
    const double TIME_INTERVEL = 2;
}

PluginLegacySSLNetworkOutput::PluginLegacySSLNetworkOutput(
    FrameBuffer * _fb,
    RoboCupSSLServer * ds_udp_server_old,
    const CameraParameters& camera_params,
    const RoboCupField& field) :
    VisionPlugin(_fb),
    _camera_params(camera_params),
    _field(field),
    _ds_udp_server_old(ds_udp_server_old) {
//    legacy_network_output_settings = new PluginLegacySSLNetworkOutputSettings();
    isRecord = true;
    save_path = "/home/zjunlict-vision-1/luckky/test_pic";
//    save_path = "./test";
//    connect(legacy_network_output_settings->isRecord,
//            SIGNAL(wasEdited(VarType *)),
//            this,
//            SLOT(RefreshNetworkOutput()));
//    connect(legacy_network_output_settings->save_path,
//            SIGNAL(wasEdited(VarType *)),
//            this,
//            SLOT(RefreshNetworkOutput()));
}

PluginLegacySSLNetworkOutput::~PluginLegacySSLNetworkOutput() {}

bool PluginLegacySSLNetworkOutput::DoubleSizeToSingleSize(
    const bool primary_field,
    const float x_in,
    const float y_in,
    const float a_in,
    float* x_out,
    float* y_out,
    float* a_out) {
  const float half_field_length =
      primary_field ?
      -min(GetFieldLine("TopTouchLine", _field.field_lines)->
              p1_x->getDouble(),
           GetFieldLine("TopTouchLine", _field.field_lines)->
              p2_x->getDouble()) :
      max(GetFieldLine("TopTouchLine", _field.field_lines)->p1_x->getDouble(),
          GetFieldLine("TopTouchLine", _field.field_lines)->p2_x->getDouble());
  _field.field_length->getDouble();
  const float line_width =
      GetFieldLine("HalfwayLine", _field.field_lines)->thickness->getDouble();
  // Ignore robots not within the referee margin minus robot radius
  if (primary_field) {
    if (x_in > 425.0 - 90.0) return false;
    *y_out = -(x_in + 0.5 * half_field_length - 0.5 * line_width);
  } else {
    if (x_in < -425.0 + 90.0) return false;
    *y_out = -(x_in - 0.5 * half_field_length + 0.5 * line_width);
  }
  *x_out = y_in;
  *a_out = angle_mod(a_in - RAD(90.0));
  return true;
}

ProcessResult PluginLegacySSLNetworkOutput::process(
    FrameData * data, RenderOptions * options) {
  (void)options;
//    qDebug() << "vision msg fuck running !";
  if (data==0) return ProcessingFailed;

  SSL_DetectionFrame * detection_frame = 0;
  detection_frame=(SSL_DetectionFrame *)data->map.get("ssl_detection_frame");

//  RefreshNetworkOutput();
//  qDebug() << "for debug" << isRecord << "clock:" << record_time << "is clock:" <<  ((clock() - record_time) / CLOCKS_PER_SEC > TIME_INTERVEL);
//  qDebug() << "clock:" << clock() << "record_time:" << record_time << "result:" << (clock() - record_time) / CLOCKS_PER_SEC ;
  if (isRecord && (clock() - record_time) / CLOCKS_PER_SEC > TIME_INTERVEL ) {
      record_time = clock();
      string pic_name = "dl" + QDateTime::currentDateTime().toString("yyyy-MM-dd-HH-mm-ss").toStdString();
      // save image
      rgbImage temp;
      VisualizationFrame * vis_frame=(VisualizationFrame *)(data->map.get("vis_frame"));
      if (vis_frame !=0 && vis_frame->valid) {
        temp.copy ( vis_frame->data );
      }
      if ( temp.getWidth() > 1 && temp.getHeight() > 1 ) {
          if ( !temp.save ( save_path +"/imgs/" + pic_name + ".png" ) ) {
              qDebug() << "save image faild!plz check in time";
          }
      }


      // save annotation via detection
      XMLDocument doc;
      XMLElement *xml_root, *xml_object;
      QString xml_int_path = QString::fromStdString(save_path + "/template.xml");
      doc.LoadFile(xml_int_path.toLatin1());
      xml_root = doc.RootElement();
      xml_object = xml_root->FirstChildElement("object");
      xml_object->DeleteChildren();

      int team = 2;
      int ball_size = detection_frame->balls_size();
      int robot_size[team];
      robot_size[0] = detection_frame->robots_blue_size();
      robot_size[1] = detection_frame->robots_yellow_size();
      SSL_DetectionBall vis_ball;
      SSL_DetectionRobot vis_robot;
      XMLElement *ball,*robot, *bndbox, *xmax, *xmin, *ymax, *ymin,
                 *orientation, *id, *color;
      qDebug() << "robot_size:" << robot_size[0] << robot_size[1];

      // ball
      for (int i = 0; i < ball_size; i++) {
          vis_ball = detection_frame->balls(i);
          ball = doc.NewElement("name");
          ball->SetText("ball");
          xml_object->InsertEndChild(ball);
          bndbox = doc.NewElement("bndbox");
          xmax = doc.NewElement("xmax");
          xmin = doc.NewElement("xmin");
          ymax = doc.NewElement("ymax");
          ymin = doc.NewElement("ymin");
          //todo
          xmax->SetText(vis_ball.x());
          xmin->SetText(vis_ball.x());
          ymax->SetText(vis_ball.x());
          ymin->SetText(vis_ball.x());
          bndbox->InsertEndChild(xmax);
          bndbox->InsertEndChild(xmin);
          bndbox->InsertEndChild(ymax);
          bndbox->InsertEndChild(ymin);
          xml_object->InsertEndChild(bndbox);
      }
      // robot
      for (int t = 0; t < team; t++) {
          for (int i = 0; i < robot_size[t]; i++) {
              vis_robot = t == 0 ? detection_frame->robots_blue(i) : detection_frame->robots_yellow(i);
              xml_object = doc.NewElement("object");
              xml_root->InsertEndChild(xml_object);
              robot = doc.NewElement("name");
              robot->SetText("robot");
              bndbox = doc.NewElement("bndbox");
              xmax = doc.NewElement("xmax");
              xmin = doc.NewElement("xmin");
//              ymax = doc.NewElement("ymax");
//              ymin = doc.NewElement("ymin");
//              //todo
              xmax->SetText(vis_robot.x());
              xmin->SetText(vis_robot.y());
//              ymax->SetText(0);
//              ymin->SetText(0);
              bndbox->InsertEndChild(xmax);
              bndbox->InsertEndChild(xmin);
//              bndbox->InsertEndChild(ymax);
//              bndbox->InsertEndChild(ymin);
              orientation = doc.NewElement("orientation");
              orientation->SetText(vis_robot.orientation());
              color = doc.NewElement("team");
              color->SetText(t);
              xml_object->InsertEndChild(robot);
              xml_object->InsertEndChild(bndbox);
              xml_object->InsertEndChild(orientation);
              xml_object->InsertEndChild(color);
          }
      }
      QString convert = QString::fromStdString(save_path + "/anno/" + pic_name + ".xml");
      doc.SaveFile(convert.toLatin1());
  }
//  qDebug() << "vision msg fuck running !";
  if (detection_frame != 0) {
    detection_frame->set_t_capture(data->time);
    detection_frame->set_frame_number(data->number);
    detection_frame->set_camera_id(_camera_params.additional_calibration_information->camera_index->getInt());
    detection_frame->set_t_sent(GetTimeSec());
    // The double-sized field server uses the normal field coordinates.
    _ds_udp_server_old->sendLegacyMessage(*detection_frame);
  }
  return ProcessingOk;
}

string PluginLegacySSLNetworkOutput::getName() {
  return "Legacy Network Output";
}

//void PluginLegacySSLNetworkOutput::RefreshNetworkOutput() {
//    isRecord = legacy_network_output_settings->isRecord->getBool();
//    save_path = legacy_network_output_settings->isRecord->getString();
//}

PluginLegacySSLNetworkOutputSettings::PluginLegacySSLNetworkOutputSettings()
{
  settings = new VarList("Legacy Network Output");

  settings->addChild(
      multicast_address = new VarString("Multicast Address","224.5.23.2"));
  settings->addChild(ds_multicast_port_old =
      new VarInt("Legacy Double-Size Field Multicast Port",10005,1,65535));
  settings->addChild(
      multicast_interface = new VarString("Multicast Interface",""));
//  settings->addChild(
//      isRecord = new VarBool("isRecord", false));
//  settings->addChild(
//      save_path = new VarString("save_path", "/home/zjunlict-vision-1/luckky/test_pic"));
}

VarList * PluginLegacySSLNetworkOutputSettings::getSettings()
{
  return settings;
}
