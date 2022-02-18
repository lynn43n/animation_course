#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

using namespace std;

class SandBox : public igl::opengl::glfw::Viewer
{
public:

	int counter = 0;

	SandBox();
	~SandBox();
	
	void Init(const std::string& config);
	double doubleVariable;
	void calc_all_weights();
	Eigen::VectorXd create_weight_vec(double w1, double w1_ind, double w2, double w2_ind);
	void calc_next_pos();
	double calc_related_distance(int i);

	void add_weights();
	void updateMovement();
	void levelk();

	Eigen::Vector3d target_pose;

	int scale;
	int joints_num;
	std::vector<Eigen::Vector3d>skeleton;
	std::vector<Movable> Joints;
	bool up;
	bool down;
	bool right;
	bool left;
	bool in;
	bool out;


	typedef
		std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
		RotationList;

	Eigen::MatrixXd V, W, C, U, M;
	Eigen::MatrixXi F, BE;
	Eigen::VectorXi P;
	RotationList vQ;
	std::vector<Eigen::Vector3d> vT;


	RotationList origin_vQ;
	std::vector<Eigen::Vector3d> origin_vT;
	std::vector<Eigen::Vector3d>origin_skeleton;

private:
	
	void Animate();
};
