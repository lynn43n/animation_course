#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>



SandBox::SandBox()
{


}
void SandBox::Init(const std::string& config)
{

	load_mesh_from_file("C:/Users/alina/source/repos/EngineForAnimationCourse/tutorial/data/sphere.obj");

	Eigen::RowVector3d center(5, 0, 0);
	parents.push_back(-1);
	data().show_overlay_depth = false;
	data().point_size = 10;
	data().line_width = 2;
	data().set_visible(false, 1);
	data().MyTranslateInSystem(data().GetRotation(), center);

	MyTranslateInSystem(GetRotation(), Eigen::RowVector3d(0, 0, -10));
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));


}
SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{

		animateCCD();

	}
}


