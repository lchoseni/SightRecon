//
// Created by pidan1231239 on 18-6-23.
//

#include "SSLAM/BA.h"

namespace sslam {

void BA::loadMap() {
#ifdef DEBUG
  cout << "BA::loadMap: " << endl;
#endif
  for (auto &mapPoints: map->mapPoints) {
    mapPointsPos[mapPoints] = mapPoints->getPosMatx13<double>();
  }

  for (auto &frame: map->frames) {
    auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
    auto t = frame->Tcw.translation();
    frameExtrinsics[frame] = cv::Matx23d(angleAxis(0), angleAxis(1), angleAxis(2),
                                         t[0], t[1], t[2]);
    if (cameraIntrinsics.find(frame->cam_) == cameraIntrinsics.end())
      cameraIntrinsics[frame->cam_] = cv::Matx14d(
          frame->cam_->fx_, frame->cam_->fy_, frame->cam_->cx_, frame->cam_->cy_);
  }
#ifdef DEBUG
  cout << "\t" << mapPointsPos.size() << " map points" << endl;
/*            int i = 0;
      for (auto &mapPoints:mapPointsPos) {
          cout << mapPoints.second << endl;
          ++i;
          if (i >= 5)break;
      }
      cout << "..." << endl;*/

  cout << "\t" << frameExtrinsics.size() << " frames" << endl;
/*            i = 0;
      for (auto &frame:frameExtrinsics) {
          cout << frame.second << endl;
          ++i;
          if (i >= 5)break;
      }
      cout << "..." << endl;*/

  cout << "\t" << cameraIntrinsics.size() << " cameras" << endl;
/*            i = 0;
      for (auto &camera:cameraIntrinsics) {
          cout << camera.second << endl;
          ++i;
          if (i >= 5)break;
      }
      cout << "..." << endl;*/
#endif
}

void BA::bundleAdjustment() {
#ifdef DEBUG
  cout << "BA::bundleAdjustment: " << endl;
#endif
  ceres::Problem problem;

#ifdef DEBUG
  cout << "\tloading frameExtrinsics..." << endl;
#endif
  for (auto &frameExtrinsic: frameExtrinsics)
    problem.AddParameterBlock(frameExtrinsic.second.val, 6);
  problem.SetParameterBlockConstant(frameExtrinsics[map->frames.front()].val);

#ifdef DEBUG
  cout << "\tloading cameraIntrinsics..." << endl;
#endif
  for (auto &cameraIntrinsic: cameraIntrinsics)
    problem.AddParameterBlock(cameraIntrinsic.second.val, 4);

#ifdef DEBUG
  cout << "\tloading mapPointsPos..." << endl;
#endif
  ceres::LossFunction *lossFunction = new ceres::HuberLoss(4);
  for (auto &mapPointPos: mapPointsPos) {
    for (auto &observedFrame: mapPointPos.first->observedFrames) {
      ceres::CostFunction *costFunction =
          new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 6, 3>(
              new ReprojectCost(observedFrame.second));
      problem.AddResidualBlock(
          costFunction,
          lossFunction,
          cameraIntrinsics[observedFrame.first->cam_].val,            // Intrinsic
          frameExtrinsics[observedFrame.first].val,  // View Rotation and Translation
          mapPointPos.second.val          // Point in 3D space
      );
    }
  }

#ifdef DEBUG
  cout << "\tsolving BA..." << endl;
#endif

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_config_options, &problem, &summary);

  if (!summary.IsSolutionUsable()) {
    std::cout << "Bundle Adjustment failed." << std::endl;
  } else {
    // Display statistics about the minimization
    std::cout << "Bundle Adjustment statistics (approximated RMSE):\n"
              << "    #views: " << frameExtrinsics.size() << "\n"
              << "    #residuals: " << summary.num_residuals << "\n"
              << "    Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
              << "    Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
              << "    Time (s): " << summary.total_time_in_seconds << "\n";
  }
}

void BA::writeMap() {
  for (auto &mapPointPos: mapPointsPos) {
    mapPointPos.first->setPos(mapPointPos.second);
  }
  for (auto &frameExtrinsic: frameExtrinsics) {
    frameExtrinsic.first->setTcw(frameExtrinsic.second);
  }
  for (auto &cameraIntrinsic: cameraIntrinsics) {

    cameraIntrinsic.first->fx_ = cameraIntrinsic.second(0);
    cameraIntrinsic.first->fy_ = cameraIntrinsic.second(1);
    cameraIntrinsic.first->cx_ = cameraIntrinsic.second(2);
    cameraIntrinsic.first->cy_ = cameraIntrinsic.second(3);
  }
}

void BA::clear() {
  map = nullptr;

  cameraIntrinsics.clear();
  frameExtrinsics.clear();
  mapPointsPos.clear();
}

BA::BA() {
  ceres_config_options.minimizer_progress_to_stdout = false;
  ceres_config_options.logging_type = ceres::SILENT;
  ceres_config_options.num_threads = 1;
  ceres_config_options.preconditioner_type = ceres::JACOBI;
  ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
}

void BA::operator()(GMap::Ptr &map) {
  this->map = map;
  loadMap();
  bundleAdjustment();
  writeMap();
  clear();
}

}