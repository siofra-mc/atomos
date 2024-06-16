#include "Application.hpp"
#include "physics/Physics.hpp"


constexpr uint16_t SIMULATION_WINDOW_WIDTH = 800;
constexpr uint16_t SIMULATION_WINDOW_HEIGHT = 800;

class BallsApp : public Application { 
	PhysicsController* physics = 0;

public:
	BallsApp() : Application(1920, 1080) {}

	virtual void onCreate() {
		physics = new PhysicsController(SIMULATION_WINDOW_WIDTH, SIMULATION_WINDOW_HEIGHT);
	}

	void onDisplay() {
		physics->displaySimulation();
	}

	void onUpdate(float deltaTime) {
		physics->update(deltaTime);
	}
};

int main() {
    BallsApp app;
	app.run();
	return 0;
}