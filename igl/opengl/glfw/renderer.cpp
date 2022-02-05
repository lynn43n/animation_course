#include "igl/opengl/glfw/renderer.h"
#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <igl/opengl/glfw/Viewer.h>
//#include <Eigen/Dense>

#include <cstdlib>

Eigen::Matrix3d C;
Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;
}

IGL_INLINE void Renderer::draw(GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window, width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}
	int coreIndx = 1;
	if (menu)
	{
		menu->pre_draw();
		menu->callback_draw_viewer_menu();
	}
	for (auto& core : core_list)
	{
		int indx = 0;
		for (auto& mesh : scn->data_list)
		{

			if (mesh.is_visible & core.id)
			{// for kinematic chain change scn->MakeTrans to parent matrix
				//Eigen::
				//  sceneAndParents = (scn->MakeTransScale() * scn->data_list[mesh.id].ParentTrans());
				//core.draw(sceneAndParents, mesh);

				//std::cout << "pick: " << scn->isPicked << std::endl;
				//std::cout << "scene: " << scn->scene_selected << std::endl;
				//std::cout << "index: " << scn->selected_data_index << std::endl;
				core.draw(scn->MakeTransScale() * scn->CalcParentsTrans(indx).cast<float>(), mesh);

				//****
				/*
				Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
				if (mesh.id > 1)
					temp = (scn->data_list[mesh.id].ParentTrans().cast<float>());
				core.draw(scn->MakeTransScale() * temp, mesh);
				*/

				//scn->data_list[3].ParentTrans();
				/*
				igl::AABB<Eigen::MatrixXd, 3>* node1 = &scn->data_list[0].kd_tree;
				igl::AABB<Eigen::MatrixXd, 3>* node2 = &scn->data_list[1].kd_tree;
				if (checkCollisionRec(node1, node2) ) {
						std::cout << "detec_collistion" << std::endl;
						isMovable = false;


				}
				else {
					if (isMovable) {
						//GetScene()->data_list[0].MyTranslateInSystem(GetScene()->GetRotation(), moveDir);
					}

				}
				*/
			}
			indx++;
		}


	}
	if (menu)
	{
		menu->post_draw();

	}

}


IGL_INLINE bool Renderer::is_collistion() {
	igl::AABB<Eigen::MatrixXd, 3>* obj1 = &scn->data_list[0].kd_tree;
	igl::AABB<Eigen::MatrixXd, 3>* obj2 = &scn->data_list[1].kd_tree;

	Eigen::Vector4d A_pos = scn->data_list[0].MakeTransScaled() * Eigen::Vector4d(0, -0.35, 0, 1);
	Eigen::Vector4d B_pos = scn->data_list[1].MakeTransScaled() * Eigen::Vector4d(0, 0, 0, 1);
	float distance = (B_pos - A_pos).norm();
	std::cout << distance << std::endl;
	return (distance <= obj1->m_box.sizes()[0]);
}


//ass 4
void Renderer::showCorrectMenu() {
	if (scn->isCollisionSnake) {
		//show Lost menu
		menu->callback_draw_viewer_window();
		scn->gameLost = true;
	}
	else if (scn->isNextLevel) {
		//show start level menu
		scn->level++;
		menu->callback_draw_custom_window();
		scn->isNextLevel = false;
		
	}
	else if (scn->start) {
		//show start menu
		menu->callback_draw_custom_window();
		//scn->start = false;
	}
	else {
		//no menu 
	}
}
bool Renderer::checkCollisionRec(igl::AABB<Eigen::MatrixXd, 3>* node1, igl::AABB<Eigen::MatrixXd, 3>* node2) {
	if (detec_collistion(node1->m_box, node2->m_box))
	{
		//No children, this is a leaf! populate field
		if (node1->is_leaf() && node2->is_leaf())
		{
			//std::cout << "drwa1" << std::endl;
			scn->data_list[0].drawBox(node1->m_box, 1);
			scn->data_list[1].drawBox(node2->m_box, 1);
			//std::cout << "draw2" << std::endl;
			return true;
		}
		else {
			//Children pointers
			igl::AABB<Eigen::MatrixXd, 3>* left1 = node1->is_leaf() ? node1 : node1->m_left;
			igl::AABB<Eigen::MatrixXd, 3>* right1 = node1->is_leaf() ? node1 : node1->m_right;
			igl::AABB<Eigen::MatrixXd, 3>* left2 = node2->is_leaf() ? node2 : node2->m_left;;
			igl::AABB<Eigen::MatrixXd, 3>* right2 = node2->is_leaf() ? node2 : node2->m_right;

			if (checkCollisionRec(left1, left2))
				return true;
			else if (checkCollisionRec(left1, right2))
				return true;
			else if (checkCollisionRec(right1, left2))
				return true;
			else if (checkCollisionRec(right1, right2))
				return true;
			else return false;
		}
	}

	return false;
}

IGL_INLINE bool Renderer::detec_collistion(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2) {
	double a0 = box1.sizes()[0] / 2, a1 = box1.sizes()[1] / 2, a2 = box1.sizes()[2] / 2,
		b0 = box2.sizes()[0] / 2, b1 = box2.sizes()[1] / 2, b2 = box2.sizes()[2] / 2,
		R0, R1, R;
	Eigen::Matrix3d A, B, C;
	Eigen::Vector3d D, C0, C1;
	Eigen::RowVector3d A0 = scn->data_list[0].GetRotation() * Eigen::Vector3d(1, 0, 0),
		A1 = scn->data_list[0].GetRotation() * Eigen::Vector3d(0, 1, 0),
		A2 = scn->data_list[0].GetRotation() * Eigen::Vector3d(0, 0, 1),
		B0 = scn->data_list[1].GetRotation() * Eigen::Vector3d(1, 0, 0),
		B1 = scn->data_list[1].GetRotation() * Eigen::Vector3d(0, 1, 0),
		B2 = scn->data_list[1].GetRotation() * Eigen::Vector3d(0, 0, 1);
	A << Eigen::RowVector3d(A0[0], A1[0], A2[0]), Eigen::RowVector3d(A0[1], A1[1], A2[1]), Eigen::RowVector3d(A0[2], A1[2], A2[2]);
	B << Eigen::RowVector3d(B0[0], B1[0], B2[0]), Eigen::RowVector3d(B0[1], B1[1], B2[1]), Eigen::RowVector3d(B0[2], B1[2], B2[2]);
	C = A.transpose() * B;

	Eigen::Vector4f C0_4cord = scn->data_list[0].MakeTransScale() * Eigen::Vector4f(box1.center()[0], box1.center()[1], box1.center()[2], 1);
	Eigen::Vector4f C1_4cord = scn->data_list[1].MakeTransScale() * Eigen::Vector4f(box2.center()[0], box2.center()[1], box2.center()[2], 1);

	C0 = Eigen::Vector3d(C0_4cord[0], C0_4cord[1], C0_4cord[2]);
	C1 = Eigen::Vector3d(C1_4cord[0], C1_4cord[1], C1_4cord[2]);

	D = C1 - C0;

	//Table case 1
	R0 = a0;
	R1 = (b0 * abs(C(0, 0))) + (b1 * abs(C(0, 1))) + (b2 * abs(C(0, 2)));
	R = abs(A0.dot(D));

	if (R > R0 + R1) return false;

	//Table case 2
	R0 = a1;
	R1 = (b0 * abs(C(1, 0))) + (b1 * abs(C(1, 1))) + (b2 * abs(C(1, 2)));
	R = abs(A1.dot(D));

	if (R > R0 + R1) return false;

	//Table case 3
	R0 = a2;
	R1 = (b0 * abs(C(2, 0))) + (b1 * abs(C(2, 1))) + (b2 * abs(C(2, 2)));
	R = abs(A2.dot(D));

	if (R > R0 + R1) return false;

	//Table case 4
	R0 = (a0 * abs(C(0, 0))) + (a1 * abs(C(1, 0))) + (a2 * abs(C(2, 0)));
	R1 = b0;
	R = abs(B0.dot(D));

	if (R > R0 + R1) return false;

	//Table case 5
	R0 = (a0 * abs(C(0, 1))) + (a1 * abs(C(1, 1))) + (a2 * abs(C(2, 1)));
	R1 = b1;
	R = abs(B1.dot(D));

	if (R > R0 + R1) return false;

	//Table case 6
	R0 = (a0 * abs(C(0, 2))) + (a1 * abs(C(1, 2))) + (a2 * abs(C(2, 2)));
	R1 = b2;
	R = abs(B2.dot(D));

	if (R > R0 + R1) return false;

	//Table case 7
	R0 = (a1 * abs(C(2, 0))) + (a2 * abs(C(1, 0)));
	R1 = (b1 * abs(C(0, 2))) + (b2 * abs(C(0, 1)));
	R = abs((C(1, 0) * A2).dot(D) - (C(2, 0) * A1).dot(D));

	if (R > R0 + R1) return false;

	//Table case 8
	R0 = (a1 * abs(C(2, 1))) + (a2 * abs(C(1, 1)));
	R1 = (b0 * abs(C(0, 2))) + (b2 * abs(C(0, 0)));
	R = abs((C(1, 1) * A2).dot(D) - (C(2, 1) * A1).dot(D));

	if (R > R0 + R1) return false;

	//Table case 9
	R0 = (a1 * abs(C(2, 2))) + (a2 * abs(C(1, 2)));
	R1 = (b0 * abs(C(0, 1))) + (b1 * abs(C(0, 0)));
	R = abs((C(1, 2) * A2).dot(D) - (C(2, 2) * A1).dot(D));

	if (R > R0 + R1) return false;

	//Table case 10
	R0 = (a0 * abs(C(2, 0))) + (a2 * abs(C(0, 0)));
	R1 = (b1 * abs(C(1, 2))) + (b2 * abs(C(1, 1)));
	R = abs((C(2, 0) * A0).dot(D) - (C(0, 0) * A2).dot(D));

	if (R > R0 + R1) return false;

	//Table case 11
	R0 = (a0 * abs(C(2, 1))) + (a2 * abs(C(0, 1)));
	R1 = (b0 * abs(C(1, 2))) + (b2 * abs(C(1, 0)));
	R = abs((C(2, 1) * A0).dot(D) - (C(0, 1) * A2).dot(D));

	if (R > R0 + R1) return false;

	//Table case 12
	R0 = (a0 * abs(C(2, 2))) + (a2 * abs(C(0, 2)));
	R1 = (b0 * abs(C(1, 1))) + (b1 * abs(C(1, 0)));
	R = abs((C(2, 2) * A0).dot(D) - (C(0, 2) * A2).dot(D));

	if (R > R0 + R1) return false;

	//Table case 13
	R0 = (a0 * abs(C(1, 0))) + (a1 * abs(C(0, 0)));
	R1 = (b1 * abs(C(2, 2))) + (b2 * abs(C(2, 1)));
	R = abs((C(0, 0) * A1).dot(D) - (C(1, 0) * A0).dot(D));

	if (R > R0 + R1) return false;

	//Table case 14
	R0 = (a0 * abs(C(1, 1))) + (a1 * abs(C(0, 1)));
	R1 = (b0 * abs(C(2, 2))) + (b2 * abs(C(2, 0)));
	R = abs((C(0, 1) * A1).dot(D) - (C(1, 1) * A0).dot(D));

	if (R > R0 + R1) return false;

	//Table case 15
	R0 = (a0 * abs(C(1, 2))) + (a1 * abs(C(0, 2)));
	R1 = (b0 * abs(C(2, 1))) + (b1 * abs(C(2, 0)));
	R = abs((C(0, 2) * A1).dot(D) - (C(1, 2) * A0).dot(D));

	if (R > R0 + R1) return false;

	return true;
}


double Renderer::calc_test_b(Eigen::Vector3d b, int l)
{
	double ans = 0;
	for (int i = 0; i <= 2; i++)
		ans += b(i) * C(l, i);
	return ans;
}
double Renderer::calc_test_a(Eigen::Vector3d a, int l)
{
	double ans = 0;
	for (int i = 0; i <= 2; i++)
		ans += a(i) * abs(C(i, l));
	return ans;
}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer, int coresNum, igl::opengl::glfw::imgui::ImGuiMenu* _menu)
{
	scn = viewer;

	doubleVariable = 0;
	core().init();
	menu = _menu;
	core().align_camera_center(scn->data().V, scn->data().F);

	if (coresNum > 1)
	{
		int width = core().viewport[2];
		int height = core().viewport[3];

		core().viewport = Eigen::Vector4f(0, 0, width / 4, height);
		left_view = core_list[0].id;
		right_view = append_core(Eigen::Vector4f(width / 4, 0, width * 3 / 4, height));
		core_index(right_view - 1);
		for (size_t i = 0; i < scn->data_list.size(); i++)
		{
			core().toggle(scn->data_list[i].show_faces);
			core().toggle(scn->data_list[i].show_lines);
			core().toggle(scn->data_list[i].show_texture);
		}
		//Eigen::Vector3d v = -scn->GetCameraPosition();
		//TranslateCamera(v.cast<float>());

		core_index(left_view - 1);
	}

	if (menu)
	{
		menu->callback_draw_viewer_menu = [&]()
		{
			// Draw parent menu content
			//menu->draw_viewer_menu(scn, core_list);
			//menu->callback_draw_custom_window();
			showCorrectMenu();

		};
	}
}




void Renderer::checkCollision() {
	igl::AABB<Eigen::MatrixXd, 3>* node1 = &Renderer::scn->data_list[0].kd_tree;
	igl::AABB<Eigen::MatrixXd, 3>* node2 = &scn->data_list[1].kd_tree;
	if (Renderer::detec_collistion(node1->m_box, node2->m_box)) {
		std::cout << "detec_collistion" << std::endl;
		isMovable = false;
	}
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}


void Renderer::MouseProcessing(int button)
{

	if (scn->isPicked)
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			//float z = far + depth * (near - far);

			Eigen::Matrix4f tmpM = core().proj;
			double xToMove = -(double)xrel / core().viewport[3] * (z + 2 * near) * (far) / (far + 2 * near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] * (z + 2 * near) * (far) / (far + 2 * near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);

			if (scn->selected_data_index == 0) {
				scn->data().MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(xToMove, 0, 0));
				scn->data().MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(0, yToMove, 0));
			}
			else {
				scn->data_list[1].MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(xToMove, 0, 0));
				scn->data_list[1].MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(0, yToMove, 0));
			}

			scn->WhenTranslate();
		}
		else
		{
			scn->data().RotateInSystem(Eigen::Vector3d(1, 0, 0), yrel / 180.0);
			scn->data().RotateInSystem(Eigen::Vector3d(0, 1, 0), xrel / 180.0);

		}
	}
	else
	{
		if (button == 1)
		{
			if (scn->selected_data_index == -1) {
				scn->MyTranslate(Eigen::Vector3d(-xrel / 180.0f, 0, 0), true);
				scn->MyTranslate(Eigen::Vector3d(0, yrel / 180.0f, 0), true);
			}
			else if (scn->selected_data_index > 0) {
				scn->data(1).MyTranslate(Eigen::Vector3d(-xrel / 180.0f, 0, 0), true);
				scn->data(1).MyTranslate(Eigen::Vector3d(0, yrel / 180.0f, 0), true);
			}
			else {
				scn->data().MyTranslate(Eigen::Vector3d(-xrel / 180.0f, 0, 0), true);
				scn->data().MyTranslate(Eigen::Vector3d(0, yrel / 180.0f, 0), true);
			}
		}
		else
		{
			if (scn->selected_data_index == -1) {
				scn->MyRotate(Eigen::Vector3d(0, -1, 0), xrel / 180.0f, true);
				scn->MyRotate(Eigen::Vector3d(-1, 0, 0), yrel / 180.0f, false);
			}
			else {
				scn->data().MyRotate(Eigen::Vector3d(0, -1, 0), xrel / 180.0f, true);
				scn->data().MyRotate(Eigen::Vector3d(-1, 0, 0), yrel / 180.0f, false);
			}
		}
	}
}


void Renderer::MouseProcessing1(int button)
{

	if (button == 1)
	{
		if (scn->selected_data_index == -1) {
			scn->MyTranslate(Eigen::Vector3d(-xrel / 180.0f, 0, 0), true);
			scn->MyTranslate(Eigen::Vector3d(0, yrel / 180.0f, 0), true);
		}
		else if (scn->selected_data_index > 0) {
			scn->data(1).MyTranslate(Eigen::Vector3d(-xrel / 180.0f, 0, 0), true);
			scn->data(1).MyTranslate(Eigen::Vector3d(0, yrel / 180.0f, 0), true);
		}
		else {
			scn->data().MyTranslate(Eigen::Vector3d(-xrel / 180.0f, 0, 0), true);
			scn->data().MyTranslate(Eigen::Vector3d(0, yrel / 180.0f, 0), true);
		}
	}
	else
	{
		if (scn->selected_data_index == -1) {
			scn->MyRotate(Eigen::Vector3d(0, -1, 0), xrel / 180.0f, true);
			scn->MyRotate(Eigen::Vector3d(-1, 0, 0), yrel / 180.0f, false);
		}
		else {
			scn->data().MyRotate(Eigen::Vector3d(0, -1, 0), xrel / 180.0f, true);
			scn->data().MyRotate(Eigen::Vector3d(-1, 0, 0), yrel / 180.0f, false);
		}
	}

}

void Renderer::updateDirection(int dir) {
	double step = 0.004;
	switch (dir) {
	case 'n': // in
		moveDir = Eigen::Vector3d(0, 0, -step);
		break;
	case 'm': // out
		moveDir = Eigen::Vector3d(0, 0, step);
	case GLFW_KEY_UP:
		moveDir = Eigen::Vector3d(0, step, 0);
		break;
	case GLFW_KEY_DOWN:
		moveDir = Eigen::Vector3d(0, -step, 0);
		break;
	case GLFW_KEY_LEFT:
		moveDir = Eigen::Vector3d(-step, 0, 0);
		break;
	case GLFW_KEY_RIGHT:
		moveDir = Eigen::Vector3d(step, 0, 0);
		break;
	default: break;
	}
}


void Renderer::rotateWithKeys(int key) {
	int step = 10;
	if (scn->selected_data_index == -1) {
		switch (key) {
		case GLFW_KEY_UP:
			scn->MyRotate(Eigen::Vector3d(-1, 0, 0), step / 180.0f, false);
			break;
		case GLFW_KEY_DOWN:
			scn->MyRotate(Eigen::Vector3d(1, 0, 0), step / 180.0f, false);
			break;
		case GLFW_KEY_LEFT:
			scn->MyRotate(Eigen::Vector3d(0, 1, 0), step / 180.0f, true);
			break;
		case GLFW_KEY_RIGHT:
			scn->MyRotate(Eigen::Vector3d(0, -1, 0), step / 180.0f, true);
			break;
		default: break;
		}
	}
	else {
		switch (key) {
		case GLFW_KEY_UP:
			scn->data().MyRotate(Eigen::Vector3d(-1, 0, 0), step / 180.0f, false);
			break;
		case GLFW_KEY_DOWN:
			scn->data().MyRotate(Eigen::Vector3d(1, 0, 0), step / 180.0f, false);
			break;
		case GLFW_KEY_LEFT:
			scn->data().MyRotate(Eigen::Vector3d(0, 1, 0), step / 180.0f, true);
			break;
		case GLFW_KEY_RIGHT:
			scn->data().MyRotate(Eigen::Vector3d(0, -1, 0), step / 180.0f, true);
			break;
		default: break;
		}
	}
}

void Renderer::TranslateCamera(Eigen::Vector3f amt)
{
	core().camera_translation += amt;
}

void Renderer::RotateCamera(float amtX, float amtY)
{
	core().camera_eye = core().camera_eye + Eigen::Vector3f(0, amtY, 0);
	Eigen::Matrix3f Mat;
	Mat << cos(amtY), 0, sin(amtY), 0, 1, 0, -sin(amtY), 0, cos(amtY);
	core().camera_eye = Mat * core().camera_eye;

}

void Renderer::toggleMove()
{
	isMovable = !isMovable;
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}



double Renderer::Picking(double newx, double newy)
{
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;

	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	//std::cout << "view matrix\n" << view << std::endl;
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTransScale() * scn->CalcParentsTrans(scn->selected_data_index).cast<float>() * scn->data().MakeTransScale();
	bool picked = igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc);
	scn->isPicked = scn->isPicked | picked;
	if (picked)
	{
		Eigen::Vector3i face = scn->data().F.row(fid);
		Eigen::Matrix3d vertices;
		Eigen::Vector4f p, pp;

		vertices.col(0) = scn->data().V.row(face(0));
		vertices.col(1) = scn->data().V.row(face(1));
		vertices.col(2) = scn->data().V.row(face(2));

		p << vertices.cast<float>() * bc, 1;
		p = view * p;
		//std::cout << scn->data().V.row(face(0)) << std::endl;
		pp = core().proj * p;
		//glReadPixels(x,  y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
		z = pp(2);
		return p(2);

	}
	return 0;

}



/*
double Renderer::Picking(double newx, double newy)
{
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;

	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	bool picked = false;
	//std::cout << "view matrix\n" << view << std::endl;
	Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
	if (scn->selected_data_index > 0) {
		temp = scn->data_list[scn->selected_data_index].ParentTrans().cast<float>();
	}
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTransScale() * temp* scn->data_list[scn->selected_data_index].MakeTransScale();
		picked = igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
			core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc);

		scn->isPicked = scn->isPicked | picked;
		if (picked)
		{
			Eigen::Vector3i face = scn->data().F.row(fid);
			Eigen::Matrix3d vertices;
			Eigen::Vector4f p, pp;

			vertices.col(0) = scn->data().V.row(face(0));
			vertices.col(1) = scn->data().V.row(face(1));
			vertices.col(2) = scn->data().V.row(face(2));

			p << vertices.cast<float>() * bc, 1;
			p = view * p;
			//std::cout << scn->data().V.row(face(0)) << std::endl;
			pp = core().proj * p;
			//glReadPixels(x,  y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
			z = pp(2);
			return p(2);

		}
		return 0;
}



*/

IGL_INLINE void Renderer::resize(GLFWwindow* window, int w, int h)
{
	if (window) {
		glfwSetWindowSize(window, w / highdpi, h / highdpi);
	}
	post_resize(window, w, h);
}

IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
{
	if (core_list.size() == 1)
	{

		core().viewport = Eigen::Vector4f(0, 0, w, h);
	}
	else
	{
		// It is up to the user to define the behavior of the post_resize() function
		// when there are multiple viewports (through the `callback_post_resize` callback)
		core(left_view).viewport = Eigen::Vector4f(0, 0, w / 4, h);
		core(right_view).viewport = Eigen::Vector4f(w / 4, 0, w - (w / 4), h);

	}
	//for (unsigned int i = 0; i < plugins.size(); ++i)
	//{
	//	plugins[i]->post_resize(w, h);
	//}
	if (callback_post_resize)
	{
		callback_post_resize(window, w, h);
	}
}

IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
{
	assert(!core_list.empty() && "core_list should never be empty");
	int core_index;
	if (core_id == 0)
		core_index = selected_core_index;
	else
		core_index = this->core_index(core_id);
	assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
	return core_list[core_index];
}

IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
{
	assert(!core_list.empty() && "core_list should never be empty");
	int core_index;
	if (core_id == 0)
		core_index = selected_core_index;
	else
		core_index = this->core_index(core_id);
	assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
	return core_list[core_index];
}

IGL_INLINE bool Renderer::erase_core(const size_t index)
{
	assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
	//assert(data_list.size() >= 1);
	if (core_list.size() == 1)
	{
		// Cannot remove last viewport
		return false;
	}
	core_list[index].shut(); // does nothing
	core_list.erase(core_list.begin() + index);
	if (selected_core_index >= index && selected_core_index > 0)
	{
		selected_core_index--;
	}
	return true;
}

IGL_INLINE size_t Renderer::core_index(const int id) const {
	for (size_t i = 0; i < core_list.size(); ++i)
	{
		if (core_list[i].id == id)
			return i;
	}
	return 0;
}

IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
{
	core_list.push_back(core()); // copies the previous active core and only changes the viewport
	core_list.back().viewport = viewport;
	core_list.back().id = next_core_id;
	next_core_id <<= 1;
	if (!append_empty)
	{
		for (auto& data : scn->data_list)
		{
			data.set_visible(true, core_list.back().id);
			//data.copy_options(core(), core_list.back());
		}
	}
	selected_core_index = core_list.size() - 1;
	return core_list.back().id;
}

//IGL_INLINE void Viewer::select_hovered_core()
//{
//	int width_window, height_window = 800;
//   glfwGetFramebufferSize(window, &width_window, &height_window);
//	for (int i = 0; i < core_list.size(); i++)
//	{
//		Eigen::Vector4f viewport = core_list[i].viewport;

//		if ((current_mouse_x > viewport[0]) &&
//			(current_mouse_x < viewport[0] + viewport[2]) &&
//			((height_window - current_mouse_y) > viewport[1]) &&
//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
//		{
//			selected_core_index = i;
//			break;
//		}
//	}
//}