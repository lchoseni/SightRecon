////
//// Created by pidan1231239 on 18-6-13.
////
//
//#ifndef SFM_LEARN_SFM_H
//#define SFM_LEARN_SFM_H
//
//#include "common_include.h"
//#include "map.h"
//#include "frame.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/cvv.hpp>
//
//#include <unordered_map>
//#include "SSLAM/BA.h"
//
//
//namespace sslam {
//
//    using namespace cv;
//
//    class SFM {
//
//        GMap::Ptr map;
//        cv::Ptr<cv::Feature2D> feature2D;
//        cv::Ptr<DescriptorMatcher> matcher;
//
//        //相邻关键帧变量
//        struct KeyFrame {
//        public:
//            typedef shared_ptr<KeyFrame> Ptr;
//            Frame::Ptr frame;
//            Mat image;
//            vector<cv::KeyPoint> keyPoints;
//            Mat descriptors;
//            unordered_map<int, MapPoint::Ptr> inlierPoints;//在descriptors或keyPoints中的序号和对应的地图点
//
//            KeyFrame(const Frame::Ptr &frame, const Mat &image) :
//                    frame(frame), image(image) {}
//        };
//
//        KeyFrame::Ptr keyFrame1, keyFrame2;
//
//    public:
//        SFM(const cv::Ptr<cv::Feature2D> &feature2D,
//            const cv::Ptr<DescriptorMatcher> &matcher
//        ) :
//                feature2D(feature2D),
//                matcher(matcher),
//                map(new GMap) {}
//
//        void addImages(const vector<string> &imagesDir, Camera::Ptr camera);
//
//        ///2D-2D初始化
//
//        //对两张图片提取、匹配、筛选特征点，求解对极约束，三角化
//        void init(Mat &image1, Mat &image2, Camera::Ptr camera);
//
//        ///3D-2D求解
//
//        //添加新帧，提取、匹配、筛选和Map的特征点，PnP求解，提取、匹配、筛选和前一帧的特征点，三角化
//        void step(Mat &image, const Camera::Ptr &camera);
//
//        ///基本处理
//
//        //加载新帧
//        void pushImage(Mat &image, const Camera::Ptr &camera);
//
//        //存储新帧
//        void saveFrame();
//
//        //检测特征点，提取描述子
//        void detectAndCompute();
//
//        //匹配、筛选特征点
//        void matchWithFrameAndFilt(vector<DMatch> &matches);
//
//        //转换齐次坐标点，保存到Map
//        void convAndAddMappoints(
//                const Mat &inlierMask, const Mat &points4D, const vector<DMatch> &matches);
//
//        //筛选匹配点
//        void filtMatches(vector<DMatch> &matches);
//    };
//
//}
//
//
//#endif //SFM_LEARN_SFM_H
