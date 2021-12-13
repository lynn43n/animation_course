
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

using namespace std;

static void drawBox(igl::opengl::glfw::Viewer &viewer) {


	
	for (int i = 0; i <= 1; i++) {
		viewer.data_list[i].tree.init(viewer.data_list[i].V, viewer.data_list[i].F);
		igl::AABB<Eigen::MatrixXd, 3> tree = viewer.data_list[i].tree;
		Eigen::AlignedBox<double, 3> box = tree.m_box;
		viewer.data_list[i].drawBox(box, 0);
	}
	
}


int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Welcome");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  viewer.data_list[0].MyTranslateInSystem(viewer.GetRotation(),Eigen::Vector3d(0.7, 0, 0));// move left
  viewer.data_list[1].MyTranslateInSystem(viewer.GetRotation(),Eigen::Vector3d(-0.7, 0, 0));// move left
  viewer.data_list[0].show_lines = false;
  viewer.data_list[1].show_lines = false;
  drawBox(viewer);

  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
