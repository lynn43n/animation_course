#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
//#include "igl/opengl/Joint.h"

using namespace std;

class SandBox : public igl::opengl::glfw::Viewer
{
public:

	//project
	int counter = 0;

	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;

	void calc_all_weights();
	Eigen::VectorXd create_weight_vec(double w1, double w1_ind, double w2, double w2_ind);
	void calc_next_pos();

	void add_weights();
	double calc_related_distance(int i);

	void levelk();
	void initBoundingBoxofSnakeJoints();
	void SandBox::updateMovement();

	int scale;
	int joints_num;
	std::vector<Eigen::Vector3d>snake_skeleton;
	std::vector<Movable> Joints;
	//boolean variable for movment
	bool up;
	bool down;
	bool right;
	bool left;
	bool in;
	bool out;


	typedef
		std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
		RotationList;
	// W - weights matrix
	// BE - Edges between joints
	// C - joints positions
	// P - parents
	// M - weights per vertex per joint matrix
	// U - new vertices position after skinning

	Eigen::MatrixXd V, W, C, U, M;
	Eigen::MatrixXi F, BE;
	Eigen::VectorXi P;
	//RotationList vQ;
	//std::vector<Eigen::Vector3d> vT;


	RotationList origin_vQ;
	std::vector<Eigen::Vector3d> origin_vT;
	std::vector<Eigen::Vector3d>origin_snake_skeleton;

	void drawsnakejointBox(Eigen::AlignedBox<double, 3> box, int color);


	//end comment Project

private:
	void Animate();
};

