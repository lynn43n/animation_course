
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

using namespace std;

static void drawDotsAndLines(igl::opengl::glfw::Viewer& viewer) {
	Eigen::Matrix4d parents = Eigen::Matrix4d().Identity();
	for (int i = 1; i <= 1; i++) {
		viewer.data_list[i].MyTranslate(Eigen::Vector3d(0, 1.6, 0),true);
		viewer.data_list[i].SetCenterOfRotation(Eigen::Vector3d(0, -0.8, 0));

		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;
		if (i != 4) {
			viewer.data_list[i].add_points(Eigen::RowVector3d(0, 0.8, 0), Eigen::RowVector3d(0, 0, 255));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(0, -0.8, 0), Eigen::RowVector3d(0, 2.4, 0), Eigen::RowVector3d(0, 255, 0));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(-1.6, 0.8, 0), Eigen::RowVector3d(1.6, 0.8, 0), Eigen::RowVector3d(255, 0, 0));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(0, 0.8, -1.6), Eigen::RowVector3d(0, 0.8, 1.6), Eigen::RowVector3d(0, 0, 255));
			viewer.data_list[i].point_size = 10;
			viewer.data_list[i].line_width = 3;
		}
	}
}
int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Welcome");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  //viewer.MyTranslate(Eigen::Vector3d(0, -3, -8), true);
 // viewer.data_list[0].MyTranslate(Eigen::Vector3d(5, 0, 0),true);

  drawDotsAndLines(viewer);


  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  viewer.MyTranslate(Eigen::Vector3d(-0.5, -3.3, -9), true);
  viewer.MyRotate(Eigen::Vector3d(1, 0, 0), 80);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
