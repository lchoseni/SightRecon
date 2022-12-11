#include "srecon/viewer.h"

#include "srecon/common_include.h"

namespace srecon {

Viewer::Viewer(Map::Ptr map) : map(map) {}
void DrawImage(Frame::Ptr frame) {
  cv::imshow("img", frame->img);
  cv::waitKey(1);
}
void Viewer::run() {
  pangolin::CreateWindowAndBind("Main", 1080, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -200, -200, 0, 0, 0,
                                pangolin::AxisDirection::AxisNegY));
  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(&handler);
  while (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    d_cam.Activate(s_cam);

    for (auto& frame : map->frames) {
      DrawFrame(frame);
      // DrawImage(frame);
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

void Viewer::DrawFrame(Frame::Ptr frame) {
  const int line_width = 2.0;
  const double scale = 10.0;
  const float w = 0.5 * scale;
  const float h = 0.3 * scale;
  const float d = 0.4 * scale;

  glPushMatrix();
  Eigen::Matrix3d R = frame->R;
  Eigen::Vector3d T = frame->T;
  Eigen::Matrix3d gtR = frame->gt_R;
  Eigen::Vector3d gtT = frame->gt_T;

  vector<double> r_t_data = {
      R(0, 0), R(1, 0), R(2, 0), 0., R(0, 1), R(1, 1), R(2, 1), 0.,
      R(0, 2), R(1, 2), R(2, 2), 0., T.x(),   T.y(),   T.z(),   1.};
  vector<double> gt_r_t_data = {
      gtR(0, 0), gtR(1, 0), gtR(2, 0), 0., gtR(0, 1), gtR(1, 1), gtR(2, 1), 0.,
      gtR(0, 2), gtR(1, 2), gtR(2, 2), 0., gtT.x(),   gtT.y(),   gtT.z(),   1.};
  glMultMatrixd((GLdouble*)r_t_data.data());
  glColor3f(1, 0, 0);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, d);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, d);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, d);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, d);

  glVertex3f(w, h, d);
  glVertex3f(w, -h, d);

  glVertex3f(w, h, d);
  glVertex3f(-w, h, d);

  glVertex3f(-w, -h, d);
  glVertex3f(-w, h, d);

  glVertex3f(-w, -h, d);
  glVertex3f(w, -h, d);

  glEnd();
  glPopMatrix();
  

  glMultMatrixd((GLdouble*)gt_r_t_data.data());

  glColor3f(0.5, 0, 1);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, d);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, d);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, d);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, d);

  glVertex3f(w, h, d);
  glVertex3f(w, -h, d);

  glVertex3f(w, h, d);
  glVertex3f(-w, h, d);

  glVertex3f(-w, -h, d);
  glVertex3f(-w, h, d);

  glVertex3f(-w, -h, d);
  glVertex3f(w, -h, d);

  glEnd();
  glPopMatrix();
}

}  // namespace srecon