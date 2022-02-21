#pragma once
#include "igl/opengl/glfw/Display.h"
#include "igl/opengl/glfw/Renderer.h"
#include "sandBox.h"

void glfw_window_size(GLFWwindow* window, int width, int height)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	rndr->post_resize(window, width, height);

}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	SandBox* scn = (SandBox*)rndr->GetScene();
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if (action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
		case 'A':
		case 'a':
		{
			rndr->core().is_animating = !rndr->core().is_animating;
			break;
		}
		case 'F':
		case 'f':
		{
			scn->data().set_face_based(!scn->data().face_based);
			break;
		}
		case 'I':
		case 'i':
		{
			scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			scn->data().invert_normals = !scn->data().invert_normals;
			break;
		}
		case 'L':
		case 'l':
		{
			rndr->core().toggle(scn->data().show_lines);
			break;
		}
		case 'O':
		case 'o':
		{
			rndr->core().orthographic = !rndr->core().orthographic;
			break;
		}
		case 'T':
		case 't':
		{

			break;

		}
		case '[':
		case ']':
		{
			if (rndr->change_camera == 0)
				rndr->change_camera = 1;
			else
				rndr->change_camera = 0;

			break;
		}
		case ';':
			scn->data().show_vertid = !scn->data().show_vertid;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		case 'w':
		case 'W':
			
			if (scn->isGameStarted) {
				if (scn->up) {
					scn->up = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->down) {
					scn->down = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->left) {
					scn->left = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->right) {
					scn->right = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->out) {
					scn->out = false;
					//scn->isActive = !scn->isActive;
				}

				if (!scn->in)
					scn->in = true;
				else
					scn->in = false;
				//scn->isActive = !scn->isActive;
			}
			// end Project comment
			break;
		case 's':
		case 'S':
			if (scn->isGameStarted) {
				if (scn->up) {
					scn->up = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->down) {
					scn->down = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->left) {
					scn->left = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->right) {
					scn->right = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->in) {
					scn->in = false;
					//scn->isActive = !scn->isActive;
				}

				if (!scn->out)
					scn->out = true;
				else
					scn->out = false;
				//scn->isActive = !scn->isActive;
			}
			//end comment Project
			break;
		
		case GLFW_KEY_UP:
			//rndr->TranslateCamera(Eigen::Vector3f(0, 0.01f,0));
			if (scn->isGameStarted) {
				if (scn->right) {
					scn->right = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->down) {
					scn->down = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->left) {
					scn->left = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->in) {
					scn->in = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->out) {
					scn->out = false;
					//scn->isActive = !scn->isActive;
				}

				if (!scn->up)
					scn->up = true;
				else
					scn->up = false;
				//scn->isActive = !scn->isActive;
			}

			break;
		case GLFW_KEY_DOWN:
			//rndr->TranslateCamera(Eigen::Vector3f(0, -0.01f,0));
			if (scn->isGameStarted) {
				if (scn->up) {
					scn->up = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->left) {
					scn->left = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->right) {
					scn->right = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->in) {
					scn->in = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->out) {
					scn->out = false;
				}

				if (!scn->down)
					scn->down = true;
				else
					scn->down = false;
			}
			break;
		case GLFW_KEY_LEFT:
			if (scn->isGameStarted) {
				if (scn->up) {
					scn->up = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->down) {
					scn->down = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->right) {
					scn->right = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->in) {
					scn->in = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->out) {
					scn->out = false;
					//scn->isActive = !scn->isActive;
				}

				if (!scn->left)
					scn->left = true;
				else
					scn->left = false;
				//scn->isActive = !scn->isActive;
			}
			break;
		case GLFW_KEY_RIGHT:
			//rndr->TranslateCamera(Eigen::Vector3f(0.01f, 0, 0));
			if (scn->isGameStarted) {
				if (scn->up) {
					scn->up = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->down) {
					scn->down = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->left) {
					scn->left = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->in) {
					scn->in = false;
					//scn->isActive = !scn->isActive;
				}
				if (scn->out) {
					scn->out = false;
					//scn->isActive = !scn->isActive;
				}

				if (!scn->right)
					scn->right = true;
				else
					scn->right = false;
				//scn->isActive = !scn->isActive;
			}
			break;
			//end cpmment Project

		case ' ':
	
			if (scn->isGameStarted) {
				scn->isActive = false;//it ruined the movment
				scn->isResume = true;
			}
			break;
		case 'k':
		case 'K':
		
			break;
		case 'j':
		case 'J':
			//scn->moving_index = (scn->moving_index + 1) % 2;
			break;

		//Ass3
		case'P':
		case 'p':
		
			break;
		case 'D':
		case 'd':
			break;

		case 'H':
		case 'h':
		{
			rndr->core().toggle(scn->data().show_overlay);
			break;
		}
		//end Ass3
		default:
			Eigen::Vector3f shift;
			float scale;
			rndr->core().get_scale_and_shift_to_fit_mesh(scn->data().V, scn->data().F, scale, shift);

			std::cout << "near " << rndr->core().camera_dnear << std::endl;
			std::cout << "far " << rndr->core().camera_dfar << std::endl;
			std::cout << "angle " << rndr->core().camera_view_angle << std::endl;
			std::cout << "base_zoom " << rndr->core().camera_base_zoom << std::endl;
			std::cout << "zoom " << rndr->core().camera_zoom << std::endl;
			std::cout << "shift " << shift << std::endl;
			std::cout << "translate " << rndr->core().camera_translation << std::endl;

			break;//do nothing
		}
}


void Init(Display& display, igl::opengl::glfw::imgui::ImGuiMenu* menu)
{
	display.AddKeyCallBack(glfw_key_callback);
	
	display.AddResizeCallBack(glfw_window_size);
	menu->init(&display);
}

