#include "srecon/optimizer.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "srecon/frame.h"
#include "srecon/map_point.h"

struct ReprojectionError {
  double x, y;
  ReprojectionError(double x, double y) : x(x), y(y) {}

  template <typename T>
  bool operator()(const T* const camera_R, const T* const camera_T,
                  const T* const map_point, T* residuals) const {
    T p[3];
    ceres::QuaternionRotatePoint(camera_R, map_point, p);
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    residuals[0] = xp - T(x);
    residuals[1] = yp - T(y);
    return true;
  }

  static ceres::CostFunction* create(const double x, const double y) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(
        new ReprojectionError(x, y));
  }
};

namespace srecon {

Optimizer::Optimizer() {}

Optimizer::~Optimizer() {}

bool Optimizer::optimize(Map::Ptr map) {
  ceres::Problem problem;
  // Get all mappoints, features in 2d.
  ceres::LocalParameterization* local_parameterization =
      new ceres::QuaternionParameterization();


  set<Frame::Ptr> f_set;
  set<MapPoint::Ptr> mp_set;
  // count how many mp and frame.
  for (auto& frame : map->frames) {
    f_set.insert(frame);
    for (auto& fea : frame->features) {
      auto mp = fea->getMapPoint();
      if (mp) {
        mp_set.insert(mp);
      }
    }
  }

  // cout << f_set.size() << " " << mp_set.size() << endl;

  double rs[f_set.size()][4];
  double ts[f_set.size()][3];
  double mps[mp_set.size()][3];
  int f_count = 0, mp_count = 0;

  std::map<Frame*, double*> mrs;
  std::map<Frame*, double*> mts;
  std::map<MapPoint*, double*> mmps;
  // for each frame, add a parameter block
  // for each feature, add a residual block
  for (auto& frame : map->frames) {
    double* w_mp;
    double* r;
    double* t;
    if (mrs.find(frame.get()) == mrs.end()) {
      r = rs[f_count];
      t = ts[f_count];
      mrs.insert(make_pair(frame.get(), r));
      mts.insert(make_pair(frame.get(), t));
      f_count++;

      Eigen::Quaterniond q(frame->R);
      r[0] = q.w();
      r[1] = q.x();
      r[2] = q.y();
      r[3] = q.z();
      t[0] = frame->T.x();
      t[1] = frame->T.y();
      t[2] = frame->T.z();
      problem.AddParameterBlock(r, 4, local_parameterization);
      problem.AddParameterBlock(t, 3);
      if (frame->id == 0) {
        problem.SetParameterBlockConstant(r);
        problem.SetParameterBlockConstant(t);
      }
    } else {
      r = mrs[frame.get()];
      t = mts[frame.get()];
    }
    for (auto& fea : frame->features) {
      auto mp = fea->getMapPoint();
      if (mp && !mp->is_outliner ) {
        if (mmps.find(mp.get()) == mmps.end()) {
          w_mp = mps[mp_count];
          mmps.insert(make_pair(mp.get(), mps[mp_count]));
          w_mp[0] = mp->w_pos(0);
          w_mp[1] = mp->w_pos(1);
          w_mp[2] = mp->w_pos(2);
          mp_count++;
        } else {
          w_mp = mmps[mp.get()];
        }
        ceres::CostFunction* cost_function =
            ReprojectionError::create(fea->norm_x, fea->norm_y);
        ceres::LossFunction* lossFunction = new ceres::HuberLoss(4);
        problem.AddResidualBlock(cost_function, lossFunction /* squared loss */,
                                 r, t, w_mp);
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  DLOG(INFO) << summary.BriefReport() << "\n";
  // if (summary.termination_type == ceres::CONVERGENCE ||
  //     summary.final_cost < 5e-02) {
    for (auto& frame : f_set) {
      double* r = mrs[frame.get()];
      double* t = mts[frame.get()];
      Eigen::Quaterniond q(r[0], r[1], r[2], r[3]);
      frame->R = q.toRotationMatrix();
      frame->T = Eigen::Vector3d(t[0], t[1], t[2]);
    }
    return true;
  // } else {
  //   std::cout << "Bundle Adjustment failed." << std::endl;
  //   return false;
  // }
}
}  // namespace srecon