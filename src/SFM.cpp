////
//// Created by pidan1231239 on 18-6-13.
////
//
//#include "SSLAM/SFM.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/cvv.hpp>
//#include <opencv2/core/eigen.hpp>
//
//using namespace cv;
//
//namespace sslam {
//
//    void SFM::addImages(const vector<string> &imagesDir, Camera::Ptr camera) {
//      vector<string> images = imagesDir;
////      std::reverse(images.begin(), images.end());
//        auto imageDirIt = images.begin();
////      *imageDirIt++;
////          *imageDirIt++;
//        Mat image1 = imread(*imageDirIt++);
//        Mat image2 = imread(*imageDirIt++);
//      cv::Mat resized_left, resized_right;
////    cout << left.rows << ", " << left.cols << endl;
////    cv::imshow("s", left);
////    cv::waitKey(0);
////      cv::resize(image1, resized_left, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
////      cv::resize(image2, resized_right, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
//
////      cv::rotate(resized_right, resized_right, cv::ROTATE_90_CLOCKWISE);
////      cv::rotate(resized_left, resized_left, cv::ROTATE_90_CLOCKWISE);
//
////      init(resized_left, resized_right, camera);
//      init(image1, image2, camera);
//
//
///*#ifdef CLOUDVIEWER_DEBUG
//        map->visInCloudViewer();
//#endif*/
//
//        //3D-2D
//
//        for (; imageDirIt != images.end(); ++imageDirIt) {
//            Mat image = imread(*imageDirIt);
//
////          cv::resize(image, image, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
////          cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
//
//#ifdef DEBUG
//            cout << endl << "==============Adding image: " + *imageDirIt << "==============" << endl;
//#endif
//#ifdef CVVISUAL_DEBUGMODE
//            cvv::showImage(image, CVVISUAL_LOCATION, "Adding image: " + *imageDirIt, "");
//#endif
//
//            step(image, camera);
//        }
//
//        //输出点云信息
///*#ifdef DEBUG
//        for (auto &mapPoints:map->mapPoints) {
//            cout << "MapPoint " << mapPoints->getPosPoint3_CV<float>() << endl;
//            cout << " has " << mapPoints->observedFrames.size() << " oberved frames" << endl;
//        }
//#endif*/
//
//        //可视化初始化点云
//#ifdef CLOUDVIEWER_DEBUG
//        map->visInCloudViewer();
//#endif
//
//        //BA
//        BA ba;
//        ba(map);
//
//        //可视化初始化点云
//#ifdef CLOUDVIEWER_DEBUG
//        map->visInCloudViewer();
//#endif
//    }
//
//    void SFM::init(Mat &image1, Mat &image2, Camera::Ptr camera) {
//        //2D-2D
//#ifdef DEBUG
//        cout << endl << "==============2D-2D initializing==============" << endl;
//#endif
//        //检测特征点并匹配
//        pushImage(image1, camera);
//        saveFrame();
//        detectAndCompute();
//        pushImage(image2, camera);
//        detectAndCompute();
//        //筛选匹配点
//        vector<DMatch> matches;
//        matchWithFrameAndFilt(matches);
//
//
//        //解对极约束并三角化
//
//        vector<Point2f> matchPoints1, matchPoints2;
//        for (auto match:matches) {
//            matchPoints1.push_back(keyFrame1->keyPoints[match.queryIdx].pt);
//            matchPoints2.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
//        }
//
//        Mat essentialMatrix, inlierMask;;
//        essentialMatrix = findEssentialMat(matchPoints1, matchPoints2,
//                                           keyFrame2->frame->cam_->getFocalLength(),
//                                           keyFrame2->frame->cam_->getPrincipalPoint(),
//                                           RANSAC, 0.999, 1.0, inlierMask);
//        cout << "E is " << endl << essentialMatrix << endl;
//#ifdef DEBUG
//        int nPointsFindEssentialMat = countNonZero(inlierMask);
//        cout << "findEssentialMat: \n\t" << nPointsFindEssentialMat << " valid points, " <<
//             (float) nPointsFindEssentialMat * 100 / matchPoints1.size()
//             << "% of " << matchPoints1.size() << " points are used" << endl;
//#endif
//        //可视化用于三角化的点
//#ifdef CVVISUAL_DEBUGMODE
//        vector<DMatch> inlierMatches;
//        vector<cv::KeyPoint> inlierKeyPoints1, inlierKeyPoints2;
//        for (int i = 0; i < matches.size(); ++i) {
//            if (!inlierMask.at<uint8_t>(i, 0))
//                continue;
//            inlierMatches.push_back(matches[i]);
//            inlierMatches.back().trainIdx = inlierKeyPoints1.size();
//            inlierMatches.back().queryIdx = inlierKeyPoints2.size();
//            inlierKeyPoints1.push_back(keyFrame1->keyPoints[matches[i].queryIdx]);
//            inlierKeyPoints2.push_back(keyFrame2->keyPoints[matches[i].trainIdx]);
//        }
//        cvv::debugDMatch(keyFrame1->image, inlierKeyPoints1, keyFrame2->image, inlierKeyPoints2, inlierMatches, CVVISUAL_LOCATION,
//                         "match used in triangulation");
//
//#endif
//
//        //解frame2的R、t并计算se3,三角化
//        Mat R, t, points4D, K_in;
//      cv::eigen2cv(keyFrame2->frame->cam_->K(), K_in);
//        recoverPose(essentialMatrix, matchPoints1, matchPoints2,
//                    K_in, R, t, 100, inlierMask,
//                    points4D);
//        Eigen::Matrix3d eigenR2;
//        cv2eigen(R, eigenR2);
//        keyFrame2->frame->Tcw = SE3(
//                eigenR2,
//                Vec3 (t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
//        );
//
//
//#ifdef DEBUG
//        int nPointsRecoverPose = countNonZero(inlierMask);
//        cout << "recoverPose: \n\t" << nPointsRecoverPose << " valid points, " <<
//             (float) nPointsRecoverPose * 100 / matchPoints1.size()
//             << "% of " << matchPoints1.size() << " points are used" << endl;
///*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
//        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
//        cout << "2D-2D frame2 SE3: " << endl << keyFrame2->frame->Tcw << endl;
//        cout << "2D-2D frame2 Tcw: " << endl << keyFrame2->frame->getTcwMatCV() << endl << endl;
//        cout << "2D-2D frame2 ProjMat: " << endl << keyFrame2->frame->getTcw34MatCV() << endl << endl;*/
//#endif
//
//
//
//        //保存三角化后的点到地图
//        convAndAddMappoints(inlierMask, points4D, matches);
//
//        //可视化重投影点
//#ifdef CVVISUAL_DEBUGMODE
//
//#endif
////map->addFrame(keyFrame2->frame);
//
//        saveFrame();
//
//    }
//
//template<typename T>
//inline void AngleAxisToQuaternion(const T *angle_axis, T *quaternion) {
//  const T &a0 = angle_axis[0];
//  const T &a1 = angle_axis[1];
//  const T &a2 = angle_axis[2];
//  const T theta_squared = a0 * a0 + a1 * a1 + a2 * a2;
//
//  if (theta_squared > T(std::numeric_limits<double>::epsilon())) {
//    const T theta = sqrt(theta_squared);
//    const T half_theta = theta * T(0.5);
//    const T k = sin(half_theta) / theta;
//    quaternion[0] = cos(half_theta);
//    quaternion[1] = a0 * k;
//    quaternion[2] = a1 * k;
//    quaternion[3] = a2 * k;
//  } else { // in case if theta_squared is zero
//    const T k(0.5);
//    quaternion[0] = T(1.0);
//    quaternion[1] = a0 * k;
//    quaternion[2] = a1 * k;
//    quaternion[3] = a2 * k;
//  }
//}
//
//    void SFM::step(Mat &image, const Camera::Ptr &camera) {
//
//        pushImage(image, camera);
//
//        //检测特征点
//        detectAndCompute();
//        //提取地图的特征点
//        Mat descriptorsMap;
//        vector<Point3f> points3D;
//        for (MapPoint::Ptr &point:map->mapPoints) {
//            if (keyFrame1->frame->isInFrame(point->pos, )) {
//                points3D.push_back(point->getPosPoint3_CV<float>());
//                descriptorsMap.push_back(point->descriptor);
//            }
//        }
//#ifdef DEBUG
//        cout << "found " << points3D.size() << " 3D points in the last frame" << endl;
//#endif
//        vector<DMatch> matches;
//        matcher->match(descriptorsMap, keyFrame2->descriptors, matches, cv::noArray());
//
//
//#ifdef DEBUG
//        cout << "found " << matches.size() << " keypoints matched with 3D points" << endl;
//#endif
//
//        //筛选匹配点
//        filtMatches(matches);
//        if(matches.empty()){
//          return;
//        }
//
//        //解PnP得相机位姿
//        vector<Point2f> points2DPnP;
//        vector<Point3f> points3DPnP;
//        for (auto match:matches) {
//            points2DPnP.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
//            points3DPnP.push_back(points3D[match.queryIdx]);
//        }
//        Mat r, t, indexInliers;
//        solvePnPRansac(points3DPnP, points2DPnP, camera->getKMatxCV(),
//                       cv::noArray(), r, t, false, 100, 8.0, 0.99,
//                       indexInliers);
//        Mat R;
//        cv::Rodrigues(r, R);
//        Eigen::Matrix3d eigenR2;
//        cv2eigen(R, eigenR2);
////      cout << "R is " << R << endl << "T is " << t <<endl;
//      double angleAxis[3] =
//          {R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2)};
//      double quaternion[4], r_arr[9];
//      ceres::AngleAxisToRotationMatrix(angleAxis, r_arr);
//      cv::Mat r_by_ceres = cv::Mat(3, 3, CV_64F) ;
//      r_by_ceres.at<double>(0, 0) = r_arr[0];
//      r_by_ceres.at<double>(1, 0) = r_arr[1];
//      r_by_ceres.at<double>(2, 0) = r_arr[2];
//      r_by_ceres.at<double>(0, 1) = r_arr[3];
//      r_by_ceres.at<double>(1, 1) = r_arr[4];
//      r_by_ceres.at<double>(2, 1) = r_arr[5];
//      r_by_ceres.at<double>(0, 2) = r_arr[6];
//      r_by_ceres.at<double>(1, 2) = r_arr[7];
//      r_by_ceres.at<double>(2, 2) = r_arr[8];
//      AngleAxisToQuaternion(angleAxis, quaternion);
//      Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
//      Eigen::AngleAxis<double> a(q.toRotationMatrix());
//      cout << "q is \t" << q.toRotationMatrix() << endl << "r_by_ceres is " << r_by_ceres << endl;
////      cout << "a is " << a.axis() << " , and angelAxis is " << R.row(0) << endl;
//        keyFrame2->frame->Tcw = SE3(
//
//                SO3(eigenR2),
//                Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
//        );
//
//#ifdef DEBUG
//        cout << "solvePnPRansac: \n\t" << indexInliers.rows << " valid points, " <<
//             (float) indexInliers.rows * 100 / points2DPnP.size()
//             << "% of " << points2DPnP.size() << " points are used" << endl;
///*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
//        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
//        cout << "2D-2D frame2 SE3: " << endl << keyFrame2->frame->Tcw << endl;
//        cout << "2D-2D frame2 Tcw: " << endl << keyFrame2->frame->getTcwMatCV() << endl << endl;
//        cout << "2D-2D frame2 ProjMat: " << endl << keyFrame2->frame->getTcw34MatCV() << endl << endl;*/
//#endif
//
//        if ((float) indexInliers.rows < 30) {
//            cout << "current frame has bad matched points!" << endl;
//            return;
//        }
//
//
//        //匹配帧间特征点
//        matches.clear();
//        matchWithFrameAndFilt(matches);
//        if (matches.empty()){
//          return;
//        }
//        //通过findEssentialMat筛选异常点
//        vector<Point2f> matchPoints1, matchPoints2;
//        for (auto match:matches) {
//            matchPoints1.push_back(keyFrame1->keyPoints[match.queryIdx].pt);
//            matchPoints2.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
//        }
//        Mat inlierMask;
//        findEssentialMat(matchPoints1, matchPoints2,
//                         keyFrame2->frame->camera->getFocalLength(),
//                         keyFrame2->frame->camera->getPrincipalPoint(),
//                         RANSAC, 0.999, 1.0, inlierMask);
//#ifdef DEBUG
//        int nPointsFindEssentialMat = countNonZero(inlierMask);
//        cout << "findEssentialMat: \n\t" << nPointsFindEssentialMat << " valid points, " <<
//             (float) nPointsFindEssentialMat * 100 / matchPoints1.size()
//             << "% of " << matchPoints1.size() << " points are used" << endl;
//#endif
//
//        //三角化
//        vector<Point2f> matchPointsNorm1, matchPointsNorm2;
//        matchPointsNorm1.reserve(matches.size());
//        matchPointsNorm2.reserve(matches.size());
//        for (auto &match:matches) {
//            matchPointsNorm1.push_back(keyFrame1->frame->camera->pixel2normal(keyFrame1->keyPoints[match.queryIdx].pt));
//            matchPointsNorm2.push_back(keyFrame2->frame->camera->pixel2normal(keyFrame2->keyPoints[match.trainIdx].pt));
////          matchPointsNorm1.push_back(keyFrame1->keyPoints[match.queryIdx].pt);
////          matchPointsNorm2.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
///*#ifdef DEBUG
//            if (i < 5) {
//                cout << keyFrame1->matchPoints[i] << endl;
//                cout << matchPointsNorm1.back() << endl << endl;
//            }
//#endif*/
//        }
//        Mat points4D;
//        triangulatePoints(keyFrame1->frame->getTcw34MatCV(CV_64F), keyFrame2->frame->getTcw34MatCV(CV_64F),
//                          matchPointsNorm1, matchPointsNorm2, points4D);
////        triangulatePoints(keyFrame1->frame->getProjMatCV(), keyFrame2->frame->getProjMatCV(),
////                          keyFrame1->matchPoints, keyFrame1->matchPoints, points4D);
//
//        //转换齐次坐标点，保存到Map，并做局部BA
//        convAndAddMappoints(inlierMask, points4D, matches);
////      map->addFrame(keyFrame2->frame);
//
//        saveFrame();
//    }
//
//    void SFM::convAndAddMappoints(const Mat &inlierMask, const Mat &points4D,
//                                  const vector<DMatch> &matches) {//归一化齐次坐标点,转换Mat
//#ifdef DEBUG
//        cout << "convAndAddMappoints: " << endl;
//#endif
//        Map::Ptr localMap(new Map);
//        //建立小地图
//        if (keyFrame1 && keyFrame2) {
//            localMap->addFrame(keyFrame1->frame);
//            localMap->addFrame(keyFrame2->frame);
//        }
//#ifdef DEBUG
//        int numOldMappoints = map->mapPoints.size();
//        //cout << "showing 5 samples of 3D points:" << endl;
//#endif
//        for (int i = 0; i < points4D.cols; ++i) {
//            MapPoint::Ptr mapPoint;
//
//            //如果是outlier，跳过
//            if (!inlierMask.empty() && !inlierMask.at<uint8_t>(i, 0))
//                continue;
//
//            //获取描述子
//            Mat descriptor = keyFrame2->descriptors.row(matches[i].trainIdx);
//
//            //如果是上一帧加到地图中的点，更新描述子、加入观测帧后跳过
//            if (keyFrame1->inlierPoints.find(matches[i].queryIdx) != keyFrame1->inlierPoints.end()) {
//                mapPoint = keyFrame1->inlierPoints[matches[i].queryIdx];
//                //更新描述子
//                mapPoint->descriptor = descriptor;
//                //加入观测帧
//                mapPoint->addObervedFrame(
//                        keyFrame2->frame, keyFrame2->keyPoints[matches[i].trainIdx].pt);
//                //记录当前帧加入地图的mapPoint和特征点下标
//                keyFrame2->inlierPoints[matches[i].trainIdx] = mapPoint;
//
//            } else {
//
//                //转换齐次坐标
//                Mat x = points4D.col(i);
//
//                //向地图增加点
//                //获取颜色
//                Vec3b rgb;
//                if (keyFrame2->image.type() == CV_8UC3) {
//                    rgb = keyFrame2->image.at<Vec3b>(keyFrame2->keyPoints[matches[i].trainIdx].pt);
//                    swap(rgb[0], rgb[2]);
//                } else if (keyFrame2->image.type() == CV_8UC1) {
//                    cvtColor(keyFrame2->image.at<uint8_t>(keyFrame2->keyPoints[matches[i].trainIdx].pt),
//                             rgb,
//                             COLOR_GRAY2RGB);
//                }
//
//
//                if (x.type() == CV_32FC1) {
//                    x /= x.at<float>(3, 0); // 归一化
//                    mapPoint = MapPoint::Ptr(new MapPoint(Vector3d(x.at<float>(0, 0),
//                                                                   x.at<float>(1, 0),
//                                                                   x.at<float>(2, 0)),
//                                                          descriptor, rgb
//                    ));
//                } else if (x.type() == CV_64FC1) {
//                    x /= x.at<double>(3, 0);
//                    mapPoint = MapPoint::Ptr(new MapPoint(Vector3d(x.at<double>(0, 0),
//                                                                   x.at<double>(1, 0),
//                                                                   x.at<double>(2, 0)),
//                                                          descriptor, rgb
//                    ));
//                }
//
//                //记录当前帧加入地图的mapPoint和特征点下标
//                keyFrame2->inlierPoints[matches[i].trainIdx] = mapPoint;
//
///*#ifdef DEBUG
//            if (i < 5)
//                cout << mapPoint->pos << endl << endl;
//#endif*/
//                mapPoint->addObervedFrame(keyFrame1->frame, keyFrame1->keyPoints[matches[i].queryIdx].pt);
//                mapPoint->addObervedFrame(keyFrame2->frame, keyFrame2->keyPoints[matches[i].trainIdx].pt);
//                map->addMapPoint(mapPoint);
//                //加入到小地图
//                localMap->addMapPoint(mapPoint);
//            }
//
//
//        }
//#ifdef DEBUG
//        cout << "\t" << map->mapPoints.size() - numOldMappoints << " new 3D points added to the map" << endl
//             << "\t" << map->mapPoints.size() << " in total" << endl;
//#endif
//        BA ba;
//        ba(localMap);
//    }
//
//
//    void SFM::matchWithFrameAndFilt(vector<DMatch> &matches) {
//#ifdef DEBUG
//        cout << "matchWithFrameAndFilt keypoints: " << endl;
//#endif
//        matcher->match(keyFrame1->descriptors, keyFrame2->descriptors, matches, noArray());
//#ifdef DEBUG
//        cout << "\tfound " << matches.size() << " keypoints matched with last frame" << endl;
//#endif
//        //筛选匹配点
//        filtMatches(matches);
//
//#ifdef CVVISUAL_DEBUGMODE
//        cvv::debugDMatch(keyFrame1->image, keyFrame1->keyPoints, keyFrame2->image, keyFrame2->keyPoints, matches, CVVISUAL_LOCATION,
//                         "2D-2D points matching");
//#endif
//    }
//
//    void SFM::filtMatches(vector<DMatch> &matches) {
//        auto minMaxDis = minmax_element(
//                matches.begin(), matches.end(),
//                [](const DMatch &m1, const DMatch &m2) {
//                    return m1.distance < m2.distance;
//                });
//        if(matches.empty()){
//          return;
//        }
//        auto minDis = minMaxDis.first->distance;
//        auto maxDis = minMaxDis.second->distance;
//        vector<DMatch> goodMatches;
//        for (auto match:matches) {
//            if (match.distance <= 4 * minDis)
////            if (match.distance <= 4 * minDis)
//                goodMatches.push_back(match);
//        }
//        matches = goodMatches;
//#ifdef DEBUG
//        cout << "\tfound " << matches.size() << " good matches" << endl;
//#endif
//    }
//
//    void SFM::detectAndCompute() {
//#ifdef DEBUG
//        cout << "detectAndCompute features: " << endl;
//#endif
//        feature2D->detect(keyFrame2->image, keyFrame2->keyPoints, noArray());
//        feature2D->compute(keyFrame2->image, keyFrame2->keyPoints, keyFrame2->descriptors);
//    }
//
//    void SFM::pushImage(Mat &image, const Camera::Ptr &camera) {
//        //加载新帧
//        Frame::Ptr frame(new Frame(camera, image));
//        keyFrame2 = KeyFrame::Ptr(new KeyFrame(frame, image));
//    }
//
//    void SFM::saveFrame() {
//        map->addFrame(keyFrame2->frame);
//        keyFrame1 = keyFrame2;
//    }
//}
