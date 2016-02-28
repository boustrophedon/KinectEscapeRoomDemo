#include <iostream>
#include <cstdlib>
#include <signal.h>

#include <cmath>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chilitags.hpp>

using namespace cv;

bool protonect_shutdown =
    false; ///< Whether the running application should shut down.

void sigint_handler(int s) { protonect_shutdown = true; }

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

std::string const clues[5] = {"one", "two", "three", "four", "five"};

Mat opencvify_rgbx_frame(Mat cvmat, libfreenect2::Frame *rgbx_frame) {
  Mat img(rgbx_frame->height, rgbx_frame->width, CV_8UC4, rgbx_frame->data);
  flip(img, cvmat, 1);
  return cvmat;
}

Mat opencvify_float_frame(Mat cvmat, libfreenect2::Frame *float_frame) {
  Mat img(float_frame->height, float_frame->width, CV_32FC1, float_frame->data);
  flip(img, cvmat, 1);
  return cvmat;
}

int put_clue(cv::Mat outputimg,  float cornerx, float cornery, float depth, int id){

  std::cout << "id: " << id << std::endl;
  std::cout << "depth: " << depth << " x, y: " << cornerx << " " << cornery << std::endl;

  if (std::isinf(depth) || std::isnan(depth)) {
    depth = 100.0;
  }

	const static cv::Scalar COLOR0(0,255,0);
	const static cv::Scalar COLOR1(255,0,0);
	const static cv::Scalar COLOR2(0,0,255);
	switch(id) {
		case 0:
			cv::putText(outputimg, "B   n    h  s  a e", cv::Point(cornerx-700, cornery+150), cv::FONT_HERSHEY_SIMPLEX, (depth/100)-2, COLOR0, 10);
			return 0;
		case 1:
			cv::putText(outputimg, "  h  d t    q  r", cv::Point(cornerx-1000, cornery+300), cv::FONT_HERSHEY_SIMPLEX, (depth/100)-4, COLOR1, 10);
			return 0;
		case 2:
			cv::putText(outputimg, " e i      e   u   s", cv::Point(cornerx-300, cornery+200), cv::FONT_HERSHEY_SIMPLEX, (depth/100)-6, COLOR2, 10);
			return 0;
		case 3:
			cv::putText(outputimg, "Thanks for playing!", cv::Point(50, 780), cv::FONT_HERSHEY_SIMPLEX, 6.0f, COLOR0, 15);
			return 0;
		default:
			return 0;
  }
}

// Doing non-trivial things in signal handler is bad. If you want to pause,
// do it in another thread.
// Though libusb operations are generally thread safe, I cannot guarantee
// everything above is thread safe when calling start()/stop() while
// waitForNewFrame().
void sigusr1_handler(int s) {
  if (devtopause == 0)
    return;
  /// [pause]
  if (protonect_paused)
    devtopause->start();
  else
    devtopause->stop();
  protonect_paused = !protonect_paused;
}

int main(int argc, char *argv[]) {
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  pipeline = new libfreenect2::CpuPacketPipeline();

  std::string serial = "";
  serial = freenect2.getDefaultDeviceSerialNumber();
  int deviceId = -1;
  size_t framemax = -1;

  if (freenect2.enumerateDevices() == 0) {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  if (pipeline) {
    dev = freenect2.openDevice(serial, pipeline);
  } else {
    dev = freenect2.openDevice(serial);
  }

  if (dev == 0) {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  devtopause = dev;

  signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
  signal(SIGUSR1, sigusr1_handler);
#endif
  protonect_shutdown = false;

  int types = 0;
  types |= libfreenect2::Frame::Color;
  types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  if (!dev->start())
    return -1;

  libfreenect2::Registration *registration = new libfreenect2::Registration(
      dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  libfreenect2::Frame bigdepth(1920, 1082, 4);

  chilitags::Chilitags chilitags;
  // chilitags.setFilter(0, 0.0f);

  size_t framecount = 0;

  Mat rgbmat(1080, 1920, CV_8UC3);
  Mat depthmat(1082, 1920, CV_32FC1);

  while (!protonect_shutdown &&
         (framemax == (size_t)-1 || framecount < framemax)) {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth,
                        nullptr);

    rgbmat = opencvify_rgbx_frame(rgbmat, rgb);
    depthmat = opencvify_float_frame(depthmat, &bigdepth);

    chilitags::TagCornerMap tags = chilitags.find(rgbmat);

    for (const std::pair<int, chilitags::Quad> &tag : tags) {
      int id = tag.first;
      Matx<float, 4, 2> points = tag.second;
      float px = points(0, 0);
      float py = points(0, 1);
      float depth = depthmat.at<float>(py - 1, px);

      put_clue(rgbmat, px, py, depth, id);

      // if (tag.first == 0) {
      //   Matx<float, 4, 2> points = tag.second;
      //   int px = int(points(0, 0));
      //   int py = int(points(0, 1));

      //   zero_depth = depthmat.at<float>(py - 1, px);

      //   circle(rgbmat, Point2i(px, py), 10, Scalar(255, 0, 0), -1);

      //   put_clue(rgbmat, clues[tag.first], zero_depth/100);
      // }
      // if (tag.first == 1) {
      //   Matx<float, 4, 2> points = tag.second;
      //   int px = int(points(0, 0));
      //   int py = int(points(0, 1));

      //   one_depth = depthmat.at<float>(py - 1, px);

      //   circle(rgbmat, Point2i(px, py), 10, Scalar(0, 255, 0), -1);
      //   std::cout << one_depth << std::endl;
      //   put_clue(rgbmat, clues[tag.first], one_depth/100);
      // }
    }

    imshow("rgb", rgbmat);
    cv::waitKey(1);

    framecount++;

    listener.release(frames);
  }

  dev->stop();
  dev->close();

  delete registration;

  return 0;
}
