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
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);


	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{

		bool first = true;
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			if (first) { // data_list[0] = circle
				load_mesh_from_file(item_name);
				parents.push_back(-1);
				first = false;
			}
			else
			{
				for (int i = 0; i < 4; i++) {
					load_mesh_from_file(item_name);
					parents.push_back(-1);
				}

			}

		}
		for (size_t i = 0; i < data_list.size(); ++i) {
			data_list[i].init_mesh();
		}
		nameFileout.close();
	}

	data_list[0].MyTranslate(Eigen::Vector3d(5, 0, 0), true); // Sphere positioning
	//data_list[1].MyTranslate((Eigen::Vector3d(0, 0, -0.8)), true);
	float pos = 1.6;
	for (int i = 1; i <= 4; i++) {
		data_list[i].MyTranslate((Eigen::Vector3d(0, 0, pos)), true);
		//pos += 1.6;
		data_list[i].setCenterOfRot(Eigen::Vector3d(0, -0.8, 0));


		// Points drawing related
		data_list[i].show_overlay_depth = false;
		data_list[i].point_size = 10;
		data_list[i].line_width = 2;
		data_list[i].add_points(Eigen::RowVector3d(0, -0.8, 0), Eigen::RowVector3d(0, 0, 1));
		if (i < 4) {
			data_list[i].add_edges(Eigen::RowVector3d(-1.6, +0.8, 0), Eigen::RowVector3d(1.6, +0.8, 0), Eigen::RowVector3d(1, 0, 0)); // X axis - red
			data_list[i].add_edges(Eigen::RowVector3d(0, -0.8, 0), Eigen::RowVector3d(0, 2.4, 0), Eigen::RowVector3d(0, 1, 0)); // Y axis - green
			data_list[i].add_edges(Eigen::RowVector3d(0, +0.8, -1.6), Eigen::RowVector3d(0, +0.8, 1.6), Eigen::RowVector3d(0, 0, 1)); // Z axis - blue
		}
		if (i == 1)
			data_list[1].setParent(nullptr);
		if (i > 1) {
			Movable* parent = &data_list[i - 1];
			data_list[i].setParent(parent);
			//data_list[i].ParentTrans();
		}


	}
	//data_list[3].ParentTrans();
	//MyTranslate(Eigen::Vector3d(0, 0, -1), true);

	//data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{



	}
}


