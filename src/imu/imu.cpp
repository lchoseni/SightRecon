// #include "srecon/imu/imu.h"

// #include "srecon/parameter.h"
// #include "srecon/utils/utils.h"

// namespace srecon {

// void IMU::computeExtraRotation(vector<Frame> &image_frames, size_t start,
//                                size_t end) {
//     // For every interval between frames, construct a equation like q_c_b *
//     // q_b_k0_b_k1 = q_c_k0_c_k1 * q_c_b

//     Eigen::MatrixXd A((end - start) * 4, 4);
//     for (size_t i = start; i < end - 1; i++) {
//         Frame &frame1 = image_frames[i];
//         Frame &frame2 = image_frames[i + 1];
//         Quaterniond q_b = imu_intervals[i + 1].inte_q;
//         Quaterniond q_c(frame1.pose_.rotationMatrix().inverse() *
//                         frame2.pose_.rotationMatrix());

//         Matrix4d L, R;
//         double w = q_c.w();
//         Vector3d vec = q_c.vec();

//         // L.block<3,3>(1,1) = Matrix3d::Identity() * w + skewSymmetric(vec);
//         // L.block<3,1>(1,0) = vec;
//         // L.block<1,3>(0,1) = -vec.transpose();
//         // L(0,0) = w;

//         // w = q_b.w();
//         // vec = q_b.vec();
//         // R.block<3, 3>(1, 1) = w * Matrix3d::Identity() - skewSymmetric(vec);
//         // R.block<3, 1>(1, 0) = vec;
//         // R.block<1, 3>(0, 1) = -vec.transpose();
//         // R(0, 0) = w;

//         L.block<3, 3>(0, 0) = Matrix3d::Identity() * w + skewSymmetric(vec);
//         L.block<3, 1>(0, 3) = vec;
//         L.block<1, 3>(3, 0) = -vec.transpose();
//         L(3, 3) = w;

//         w = q_b.w();
//         vec = q_b.vec();
//         R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - skewSymmetric(vec);
//         R.block<3, 1>(0, 3) = vec;
//         R.block<1, 3>(3, 0) = -vec.transpose();
//         R(3, 3) = w;

//         A.block<4, 4>((i - 1) * 4, 0) = L - R;
//     }

//     JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
//     Matrix<double, 4, 1> x = svd.matrixV().col(3);
//     Quaterniond estimated_R(x);
//     Matrix3d ric = estimated_R.toRotationMatrix().inverse();
//     // cout << svd.singularValues().transpose() << endl;
//     // cout << ric << endl;
//     Vector3d ric_cov;
//     // ric_cov = svd.singularValues().tail<3>();
//     // if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
//     // {
//     //     calib_ric_result = ric;
//     //     return true;
//     // }
//     // else
//     //     return false;
// }

// void perpendicularBasis(Vector3d &g, MatrixXd &basis) {
//     // projection matrix with normalized vector.
//     // b - a * aT * b / aT * a
//     Vector3d b1, b2;
//     Vector3d a = g.normalized();
//     Vector3d b(0, 0, 1);

//     if (a == b) b << 0, 1, 0;
//     b1 = (b - a * (a.transpose() * b)).normalized();
//     b2 = a.cross(b1);
//     basis.block<3, 1>(0, 0) = b1;
//     basis.block<3, 1>(0, 1) = b2;
// }

// void IMU::refineGravity(vector<Frame> &image_frames, size_t start, size_t end,
//                         Vector3d &g) {
//     int columns = 3 * (end - start + 1) + 2 + 1;
//     int lines = 6 * (end - start + 1);
//     MatrixXd A(lines, columns);
//     MatrixXd b(lines);
//     A.setZero();
//     b.setZero();
//     MatrixXd tmp_A(6, 9);
//     MatrixXd tmp_b(6);
//     Matrix3d Rbk_c0;
//     Matrix3d R_bk_bk1;
//     MatrixXd g_basis(3, 2);
//     Vector3d norm_g = g.normalized() * G.norm();
//     for (size_t i = start; i < end; i++) {
//         tmp_A.setZero();
//         tmp_b.setZero();

//         IMUInterval &inte = imu_intervals[i];
//         double dt = inte.dt;
//         Frame &frame1 = image_frames[i];
//         Frame &frame2 = image_frames[i + 1];
//         Rbk_c0 = (frame1.pose_.rotationMatrix() * RIC[0]).transpose();
//         R_bk_bk1 = frame1.pose_.rotationMatrix().transpose() *
//                    frame2.pose_.rotationMatrix();
//         perpendicularBasis(g, g_basis);

//         tmp_A.block<3, 3>(0, 0) = Matrix3d::Identity() * -dt;
//         // inverse of c0_ck * c_b
//         tmp_A.block<3, 2>(0, 6) = 0.5 * dt * dt * Rbk_c0 * g_basis;
//         tmp_A.block<3, 1>(0, 8) =
//             Rbk_c0 * (frame2.pose_.translation() - frame1.pose_.translation()) /
//             100.0;
//         tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
//         tmp_A.block<3, 3>(3, 3) = R_bk_bk1;
//         tmp_A.block<3, 2>(3, 6) = Rbk_c0 * dt * g_basis;

//         tmp_b.block<3, 1>(0, 0) = inte.inte_p + TIC[0] - R_bk_bk1 * TIC[0] -
//                                   0.5 * Rbk_c0 * dt * dt * norm_g;
//         tmp_b.block<3, 1>(3, 0) = inte.inte_v - Rbk_c0 * dt * norm_g;

//         int row_start = 3 * (i - start);

//         A.block<6, 6>(row_start, row_start) += tmp_A.leftCols(6);
//         A.block<6, 3>(row_start, columns - 4) += tmp_A.rightCols(4);

//         b.segment<3>(row_start) += tmp_b.head(3);
//         b.segment<3>(row_start + 1) += tmp_b.tail(3);
//     }

//     // Big matrix mulplication may cost much time.

//     VectorXd x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
//     double s = x(lines - 1) / 100.0;
//     Vector2d dxdy = x.segment<2>(lines - 3);
//     g = (norm_g + g_basis * dxdy).normalized() * G.norm();
// }

// bool IMU::alignVisual(vector<Frame> &image_frames, size_t start, size_t end,
//                       Vector3d &g) {
//     // Compute s, v and g by Tc0ck = Tc0bkTbc
//     int columns = 3 * (end - start + 1) + 4;
//     int lines = 6 * (end - start + 1);
//     MatrixXd A(lines, columns);
//     MatrixXd b(lines);
//     A.setZero();
//     b.setZero();
//     MatrixXd tmp_A(6, 9);
//     MatrixXd tmp_b(6);
//     Matrix3d Rbk_c0;
//     Matrix3d R_bk_bk1;
//     for (size_t i = start; i < end; i++) {
//         tmp_A.setZero();
//         tmp_b.setZero();

//         IMUInterval &inte = imu_intervals[i];
//         double dt = inte.dt;
//         Frame &frame1 = image_frames[i];
//         Frame &frame2 = image_frames[i + 1];
//         Rbk_c0 = (frame1.pose_.rotationMatrix() * RIC[0]).transpose();
//         R_bk_bk1 = frame1.pose_.rotationMatrix().transpose() *
//                    frame2.pose_.rotationMatrix();
//         tmp_A.block<3, 3>(0, 0) = Matrix3d::Identity() * -dt;
//         // inverse of c0_ck * c_b
//         tmp_A.block<3, 3>(0, 6) =
//             0.5 * dt * dt * frame1.pose_.rotationMatrix() * RIC[0];
//         tmp_A.block<3, 1>(0, 9) =
//             frame1.pose_.rotationMatrix() * RIC[0] *
//             (frame2.pose_.translation() - frame1.pose_.translation()) / 100.0;
//         tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
//         tmp_A.block<3, 3>(3, 3) = R_bk_bk1;
//         tmp_A.block<3, 3>(3, 6) = Rbk_c0 * RIC[0] * dt;

//         tmp_b.block<3, 1>(0, 0) = inte.inte_p + TIC[0] - R_bk_bk1 * TIC[0];
//         tmp_b.block<3, 1>(3, 0) = inte.inte_v;

//         int row_start = 3 * (i - start);

//         A.block<6, 6>(row_start, row_start) += tmp_A.leftCols(6);
//         A.block<6, 4>(row_start, columns - 4) += tmp_A.rightCols(4);

//         b.segment<3>(row_start) += tmp_b.head(3);
//         b.segment<3>(row_start + 1) += tmp_b.tail(3);
//     }

//     VectorXd x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
//     double s = x(lines - 1) / 100.0;
//     g = x.tail<1>();

//     // Check the norm distance and scale
//     if (abs(g.norm() - G.norm()) > 1.0 || s < 0) {
//         return false;
//     }

//     refineGravity(image_frames, start, end, g);
// }

// bool IMU::computeGyroBias(vector<Frame> &image_frames, Vector3d &delta_bias_g) {
//     Matrix3d A;
//     Vector3d b;
//     if (image_frames.size() != imu_intervals.size()) {
//         cerr << "The ocunt of image frames is not the same as imu intervals.";
//         return false;
//     }
//     size_t count = image_frames.size();
//     for (size_t i = 0; i < count - 1; i++) {
//         Matrix3d tmp_A;
//         Vector3d tmp_b;
//         tmp_A.setZero();
//         tmp_b.setZero();
//         // Get the rotation matrix and the gyro change between two frames
//         // and these two matrixes should be the equal.
//         // but the truth is that the  it can't be equal. It's like a  least square
//         // problem.
//         Matrix3d frame_trans =
//             image_frames[i].pose_.rotationMatrix().transpose() *
//             image_frames[i + 1].pose_.rotationMatrix().transpose();
//         Quaterniond frame_q = Quaterniond(frame_trans);
//         Quaterniond delta_q = imu_intervals[i].inte_q;
//         tmp_b = 2 * (delta_q.inverse() * frame_q.inverse()).vec();
//         tmp_A = imu_intervals[i].jacobian.block<3, 3>(3, 12);
//         A += tmp_A.transpose() * tmp_A;
//         b += tmp_A.transpose() * tmp_b;
//     }
//     delta_bias_g = A.ldlt().solve(b);
//     return true;
//     // TODO Recompute integration based on new bias of g.
// }

// void IMUInterval::midPointIntegration(int start, int end, float &a, float &w) {
//     // Calculae mid point of radius speed and previous accelaration, transform its
//     // to imu body coordinate. Apply mid radius to the start pose to get more
//     // accurate pose at the end time, then transform end accelaratin to imu body
//     // coordinate. Finally get mid accelaration.

//     Vector3d mid_gyro = (gyro_ws[start] + gyro_ws[end]) / 2 - bias_gyro;
//     Vector3d start_acce = inte_q * (acce_as[start] - bias_a);
//     Quaterniond end_q =
//         inte_q * Quaterniond(1, 0.5 * dt * mid_gyro[0], 0.5 * dt * mid_gyro[1],
//                              0.5 * dt * mid_gyro[2]);
//     Vector3d end_acce = end_q * (acce_as[end] - bias_a);
//     Vector3d mid_acce = 0.5 * (start_acce + end_acce);

//     inte_q = end_q;
//     inte_p = inte_p + inte_v * dt + 0.5 * mid_acce * dt * dt;
//     inte_v = inte_v + mid_acce * dt;
// }

// void IMUInterval::computeJacobian(Vector3d &mid_gyro, Vector3d &acc_start,
//                                   Vector3d &acc_end, Vector3d &bias_a,
//                                   Quaterniond &pose_start,
//                                   Quaterniond &pose_end, double &dt) {
//     Vector3d w_x = mid_gyro;
//     Vector3d a_0_x = acc_start - bias_a;
//     Vector3d a_1_x = acc_end - bias_a;
//     Matrix3d R_w_x, R_a_0_x, R_a_1_x;

//     R_w_x << 0, -w_x(2), w_x(1), w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
//     R_a_0_x << 0, -a_0_x(2), a_0_x(1), a_0_x(2), 0, -a_0_x(0), -a_0_x(1),
//         a_0_x(0), 0;
//     R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1),
//         a_1_x(0), 0;

//     MatrixXd F = MatrixXd::Zero(15, 15);
//     F.block<3, 3>(0, 0) = Matrix3d::Identity();
//     F.block<3, 3>(0, 3) =
//         -0.25 * pose_start.toRotationMatrix() * R_a_0_x * dt * dt +
//         -0.25 * pose_end.toRotationMatrix() * R_a_1_x *
//             (Matrix3d::Identity() - R_w_x * dt) * dt * dt;
//     F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * dt;
//     F.block<3, 3>(0, 9) =
//         -0.25 * (pose_start.toRotationMatrix() + pose_end.toRotationMatrix()) *
//         dt * dt;
//     F.block<3, 3>(0, 12) =
//         -0.25 * pose_end.toRotationMatrix() * R_a_1_x * dt * dt * -dt;
//     F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * dt;
//     F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * dt;
//     F.block<3, 3>(6, 3) = -0.5 * pose_start.toRotationMatrix() * R_a_0_x * dt +
//                           -0.5 * pose_end.toRotationMatrix() * R_a_1_x *
//                               (Matrix3d::Identity() - R_w_x * dt) * dt;
//     F.block<3, 3>(6, 6) = Matrix3d::Identity();
//     F.block<3, 3>(6, 9) =
//         -0.5 * (pose_start.toRotationMatrix() + pose_end.toRotationMatrix()) * dt;
//     F.block<3, 3>(6, 12) =
//         -0.5 * pose_end.toRotationMatrix() * R_a_1_x * dt * -dt;
//     F.block<3, 3>(9, 9) = Matrix3d::Identity();
//     F.block<3, 3>(12, 12) = Matrix3d::Identity();
//     // cout<<"A"<<endl<<A<<endl;

//     MatrixXd V = MatrixXd::Zero(15, 18);
//     V.block<3, 3>(0, 0) = 0.25 * pose_start.toRotationMatrix() * dt * dt;
//     V.block<3, 3>(0, 3) =
//         0.25 * -pose_end.toRotationMatrix() * R_a_1_x * dt * dt * 0.5 * dt;
//     V.block<3, 3>(0, 6) = 0.25 * pose_end.toRotationMatrix() * dt * dt;
//     V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
//     V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * dt;
//     V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * dt;
//     V.block<3, 3>(6, 0) = 0.5 * pose_start.toRotationMatrix() * dt;
//     V.block<3, 3>(6, 3) =
//         0.5 * -pose_end.toRotationMatrix() * R_a_1_x * dt * 0.5 * dt;
//     V.block<3, 3>(6, 6) = 0.5 * pose_end.toRotationMatrix() * dt;
//     V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
//     V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * dt;
//     V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * dt;

//     jacobian = F * jacobian;
//     // covariance = F * covariance * F.transpose() + V * noise * V.transpose();
// }

// }  // namespace srecon