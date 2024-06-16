#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <unordered_map>
#include <GLFW/glfw3.h>
#include <glm.hpp>
#include "physics/Physics.hpp"
#include "Timer.hpp"
#include "Application.hpp"

#include <vector>
#include <sstream>
#include <iostream>


constexpr int SIMULATION_WINDOW_WIDTH = 800;
constexpr int SIMULATION_WINDOW_HEIGHT = 700;

constexpr float TIME_STEP = 1.f / 90.f;

bool debug;
Timer timer;
float frames = 0.f;

Application::Application(uint16_t width, uint16_t height) {
	this->m_screenWidth = width;
	this->m_screenHeight = height;
	init();
}

Application::~Application() {
	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

void Application::init() {

	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X

	// glfw window creation
	// --------------------
	m_window = glfwCreateWindow(m_screenWidth, m_screenHeight, "Atomos", NULL, NULL);
	if (m_window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(1);
	}
	glfwMakeContextCurrent(m_window);
	//init glew after the context have been made
	glewInit();


	glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // IF using Docking Branch

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(m_window, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
	ImGui_ImplOpenGL3_Init();
}

void Application::run() {
	onCreate();

	timer.start();
	float deltaTime = 0.f;
	while (m_window) {
		glfwPollEvents();
		processInput(m_window);
		
		onDisplayWrapper();
		// TODO: meter updates
		deltaTime = timer.readSplitMillis() / 1000;
		if (deltaTime >= TIME_STEP) {
			timer.markSplit();

			std::stringstream ss;
			ss << "Atomos - " << deltaTime << "ms";
			glfwSetWindowTitle(m_window, ss.str().c_str());

			onUpdate(deltaTime);
		}
    }
}

void Application::onDisplayWrapper() {
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	onDisplay();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(m_window);
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void Application::processInput(GLFWwindow *m_window) {
    if (glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(m_window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}
