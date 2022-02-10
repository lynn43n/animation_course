
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"
#include <imgui/imgui.cpp>

using namespace std;

static bool toggleButton(const char *id) {
	//https://github.com/ocornut/imgui/issues/1537
	static bool enable_7m = false;  // default value, the button is disabled 
	static float b = 1.0f; //  test whatever color you need from imgui_demo.cpp e.g.
	static float c = 0.5f; // 
	static int i = 3;
	bool showWindow = true;
	if (enable_7m == true)
	{

		ImGui::PushID(id);
		ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(i / 7.0f, b, b));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(i / 7.0f, b, b));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(i / 7.0f, c, c));
		ImGui::Button(id);
	
		//if (ImGui::IsItemClicked(0))
		//{
		//	enable_7m = !enable_7m;
		//}

		ImGui::PopStyleColor(3);
		ImGui::PopID();
		showWindow = false;


	}
	else
	{
		if (ImGui::Button("Let's Start"))
			enable_7m = true;
	}
	return showWindow;
}
int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Animation3D - Final Project");
	Renderer renderer;
	SandBox viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.Init("configuration.txt");
	disp->SetRenderer(&renderer);
	
	menu.callback_draw_viewer_window = [&]() {
		ImGui::CreateContext();
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(400, 320), ImGuiCond_FirstUseEver);
		static bool showWindow = true;
		if (showWindow) {
			if (!ImGui::Begin(
				"You Lost", &showWindow,
				ImGuiWindowFlags_NoSavedSettings
			)) {
				ImGui::End();
			}
			else {
				// Expose the same variable directly ...
				ImGui::PushItemWidth(-80);
				ImGui::Text("Your Score is: %d", viewer.score);
				ImGui::Text("You have reached level number: %d", viewer.level);
				ImGui::PopItemWidth();


				showWindow = toggleButton("Let's Play Again");
				//viewer.score = 0;
				//viewer.level = 1;
				viewer.start = true;

				//ImGuiWindow* window = ImGui::FindWindowByName("Let's Play");

				ImGui::End();
			}
		}

	};

	menu.callback_draw_custom_window = [&]()
	{
		ImGui::CreateContext();
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiCond_FirstUseEver);
		static bool showWindow = true;
		if (showWindow) {
			if (!ImGui::Begin(
				"Let's Play", &showWindow,
				ImGuiWindowFlags_NoSavedSettings
			)) {
				ImGui::End();
			}
			else {
				// Expose the same variable directly ...
				ImGui::PushItemWidth(-80);
				ImGui::Text("Your Score is: %d", viewer.score);
				ImGui::Text("Level Number: %d", viewer.level);
				ImGui::PopItemWidth();
				

				showWindow = toggleButton("Let's Play");

				//ImGuiWindow* window = ImGui::FindWindowByName("Let's Play");
				
				ImGui::End();
			}
		}
		
	
	};

	Init(*disp, &menu);
	renderer.init(&viewer, 3, &menu);
	renderer.selected_core_index = 1;

	disp->launch_rendering(true);
	//delete &menu;
	delete disp;
	
}
