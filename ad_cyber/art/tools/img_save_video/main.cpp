
#include "../gpu_jpeg/gpu_codec.h"
#include "cm/cm.h"

#include <sys/stat.h>
#include <unistd.h>
using namespace stoic;

std::string save_type;

namespace stoic::app::codec {

class ImageEncoder {
 public:
  ImageEncoder(int width, int height, caic_sensor::ImageEncoding type) {
    gpu_encoder_ = std::make_shared<GPUEncoder>(width, height, type);
  }
  
  void encoder_and_save(const std::shared_ptr<::caic_sensor::Image> msg, std::string topic_name) {
    int size = msg->size;

    uint8_t* im_h264 = nullptr;
    int compress_size = 0;
    gpu_encoder_->encode((uint8_t*)msg->p_data, &im_h264, &compress_size);
    printf("compress_size is: %d\n", compress_size);

    if(save_type=="video")
    {
      std::string video_name = topic_name + "_video.mp4";
      // printf("video_name is: %s\n", video_name.c_str());

      FILE* fp = fopen(video_name.c_str(), "ab+");
      fwrite((void*)im_h264, 1, compress_size, fp);
      fclose(fp);
    }
    else if(save_type=="img")
    {
      std::string save_dir = "./" + topic_name;

      if(access(save_dir.c_str(), F_OK))
      {
        mkdir((save_dir).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      }
      
      std::string img_name = save_dir + "/" + std::to_string(msg->header.stamp)+".jpg";

      FILE* fp = fopen(img_name.c_str(), "wb");
      fwrite((void*)im_h264, 1, compress_size, fp);
      fclose(fp);
    }
    else
    {
      printf("ERROR: args[2] save type %s is not supported!!! Please choose args[2]=video or args[2]=img !!!!\n", save_type.c_str());
    }

    
  }

 private:
  std::shared_ptr<GPUEncoder> gpu_encoder_;

};
}  // namespace stoic::app::codec

void callbackFrontWide(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "frontwide");
  printf("callbackFrontWide, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callbackFrontRight(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "frontright");
  printf("callbackFrontRight, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callbackFrontLeft(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "frontleft");
  printf("callbackFrontLeft, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callbackRear(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "rear");
  printf("callbackRear, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callbackRearRight(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "rearright");
  printf("callbackRearRight, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callbackRearLeft(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "rearleft");
  printf("callbackRearLeft, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callback_fish_front(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "fish_front");
  printf("callback_fish_front, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

void callback_fish_rear(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "fish_rear");
  printf("callback_fish_rear, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}
void callback_fish_left(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "fish_left");
  printf("callback_fish_left, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}
void callback_fish_right(const std::shared_ptr<::caic_sensor::Image>& msg) {
  int64_t now_us = cm::Timer::now_us();
  static stoic::app::codec::ImageEncoder encoder(msg->width, msg->height, msg->encoding);
  encoder.encoder_and_save(msg, "fish_right");
  printf("callback_fish_right, seq is: %d, time now is: %f,  time_sub is: %f\n", msg->header.seq,
         now_us / 1000.0, (now_us - msg->header.stamp) / 1000.0);
}

int main(const int argc, char** const argv) {
  if (argc != 3) {
    printf("Help: app_name [FISH/NOTFISH] [video/img] \n");
    return -1;
  }
  save_type = std::string(argv[2]);

  if (std::string(argv[1]) == "FISH") {
    printf("FISH\n");
    std::string node_name = "test_sub_img";
    std::string fish_front = "/front_fisheye/image_raw";
    std::string fish_rear = "/rear_fisheye/image_raw";
    std::string fish_left = "/left_fisheye/image_raw";
    std::string fish_right = "/right_fisheye/image_raw";
    cm::init(argc, argv, node_name);
    cm::NodeHandle node(node_name);
    cm::Subscriber sub_fish1 =
        node.subscribe<::caic_sensor::Image, true>(fish_front, 1, callback_fish_front);
    cm::Subscriber sub_fish2 =
        node.subscribe<::caic_sensor::Image, true>(fish_rear, 1, callback_fish_rear);
    cm::Subscriber sub_fish3 =
        node.subscribe<::caic_sensor::Image, true>(fish_left, 1, callback_fish_left);
    cm::Subscriber sub_fish4 =
        node.subscribe<::caic_sensor::Image, true>(fish_right, 1, callback_fish_right);

    cm::spin();

  } else if (std::string(argv[1]) == "NOTFISH") {
    printf("NOTFISH\n");
    std::string node_name = "mock_sub_img";
    std::string topic_name_fw = "/front_wide/image_raw";
    std::string topic_name_fr = "/front_right/image_raw";
    std::string topic_name_fl = "/front_left/image_raw";
    std::string topic_name_rr = "/rear_right/image_raw";
    std::string topic_name_rl = "/rear_left/image_raw";
    std::string topic_name_r = "/rear/image_raw";

    cm::init(argc, argv, node_name);
    cm::NodeHandle node(node_name);

    cm::Subscriber sub1 =
        node.subscribe<::caic_sensor::Image, true>(topic_name_fw, 1, callbackFrontWide);

    cm::Subscriber sub2 =
        node.subscribe<::caic_sensor::Image, true>(topic_name_fr, 1, callbackFrontRight);

    cm::Subscriber sub3 =
        node.subscribe<::caic_sensor::Image, true>(topic_name_fl, 1, callbackFrontLeft);

    cm::Subscriber sub4 = node.subscribe<::caic_sensor::Image, true>(topic_name_r, 1, callbackRear);

    cm::Subscriber sub5 =
        node.subscribe<::caic_sensor::Image, true>(topic_name_rr, 1, callbackRearRight);

    cm::Subscriber sub6 =
        node.subscribe<::caic_sensor::Image, true>(topic_name_rl, 1, callbackRearLeft);

    cm::spin();
  } else {
    printf("Help: app_name FISH. or app_name NOTFISH\n");
    return -1;
  }
  return 0;
}
