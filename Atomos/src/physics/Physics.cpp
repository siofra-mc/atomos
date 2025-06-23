#include "Physics.hpp"
#include "imgui.h"
#include <iostream>
#include <thread>
#include <iostream>

#include "Timer.hpp"

constexpr int IMGUI_FRAME_MARGIN = 5;
constexpr float GRAVITATIONAL_FORCE = 45.f;
constexpr float SPAWNER_EXIT_SPEED = 160.f;
constexpr float MAX_SPEED = SPAWNER_EXIT_SPEED * 3.5f;
constexpr int MAX_OBJECTS = 200;
constexpr float DENSITY = 2.f;
constexpr int COLLISION_ITERATIONS = 5;
constexpr int THREAD_COUNT = 4;
constexpr float MAX_TIME_STEP(1.f / 60.f);
constexpr glm::vec2 SPAWNER_OFFSET = glm::vec2(-OBJECT_SIZE * 2, OBJECT_SIZE * 2 + 2);

constexpr float VELOCITY_EPSILON = 0.01f;
constexpr float TIMESTEP_EPSILON = 0.0001f;


#define USE_COLLISION_GRID
#define USE_THREADS


static uint16_t objCount = 0;

PhysicsController::PhysicsObject::PhysicsObject(PhysicsController* ctrlr, glm::vec2 pos, float r, glm::vec2 v) {
	controller = ctrlr;

	position = pos;
	velocity = v;
	acceleration = glm::vec2(0);
	infrastepTime = 0.f;
	radius = r;	
	mass = r * r * DENSITY;
	

	// TODO: create Color class
	uint16_t hue = objCount % 360;
	// TODO: turn into interpolation
	double fun = 1 - abs( fmod(static_cast<float>(hue) / 60.f, 2) - 1);
	switch (hue / 60) {
	case 0:
		color = 0xFF0000FF + (static_cast<int>(0xFF * fun) << 8);
		break;
	case 1:
		color = 0xFF00FF00 + static_cast<int>(0xFF * fun);
		break;
	case 2:
		color = 0xFF00FF00 + (static_cast<int>(0xFF * fun) << 16);
		break;
	case 3:
		color = 0xFFFF0000 + (static_cast<int>(0xFF * fun) << 8);
		break;
	case 4:
		color = 0xFFFF0000 + static_cast<int>(0xFF * fun);
		break;
	case 5:
		color = 0xFF0000FF + (static_cast<int>(0xFF * fun) << 16);
		break;
	default:
		color = 0xFFFFFFFF;  
		break;
	}
	objCount++;

}

PhysicsController::PhysicsObject::~PhysicsObject() {
	objCount--;
}

void PhysicsController::PhysicsObject::nudge() {
	position += velocity * TIMESTEP_EPSILON;
	infrastepTime += TIMESTEP_EPSILON;
}

void PhysicsController::PhysicsObject::accelerate(glm::vec2 acc) {
	acceleration += acc;
}


void PhysicsController::PhysicsObject::enforceBoundaries(uint16_t width, uint16_t height) {
	if (position.y > height - radius - IMGUI_FRAME_MARGIN) {
		position.y = height - radius - IMGUI_FRAME_MARGIN;
		velocity.y *= -ELASTICITY;
	}
	if (position.y < radius + IMGUI_FRAME_MARGIN) {
		position.y = radius + IMGUI_FRAME_MARGIN;
		velocity.y *= -ELASTICITY;
	}

	if (position.x > width - radius - IMGUI_FRAME_MARGIN) {
		position.x = width - radius - IMGUI_FRAME_MARGIN;
		velocity.x *= -ELASTICITY;
	}
	if (position.x < radius + IMGUI_FRAME_MARGIN) {
		position.x = radius + IMGUI_FRAME_MARGIN;
		velocity.x *= -ELASTICITY;
	}
}


void PhysicsController::PhysicsObject::update(float timeDelta, uint16_t simWidth, uint16_t simHeight) {
	velocity += acceleration * timeDelta;
	acceleration = glm::vec2(0);
	if (glm::length(velocity) < VELOCITY_EPSILON) velocity = glm::vec2(0);
#ifndef USE_QUEUE
	position += velocity * timeDelta;
	enforceBoundaries(simWidth, simHeight);
#endif
}

size_t PhysicsController::CollisionNode::count() const {
	return numObjects;
}

/*PhysicsController::PhysicsObject* PhysicsController::CollisionNode::getObject(int8_t index) const {
	if (head == 0 || index >= numObjects) return nullptr;
	
#ifdef USE_QUEUE
	PhysicsObject* target = head;
	for (int i = 0; i < index; i++) target = target->next;
	return target;
#else
	return objects[index];
#endif
}*/

bool PhysicsController::CollisionNode::insert(PhysicsObject* obj) {
//	if (!head) {
//		head = tail = obj;
//		obj->next = obj->previous = obj;
//		numObjects++;
//		return 1;
//	}
//
//	head->previous = tail->next = obj;
//	obj->previous = tail;
//	obj->next = head;
//	tail = obj;
	contents.push_back(obj);
	numObjects++;
	return 1;
}

bool PhysicsController::CollisionNode::remove(PhysicsObject* obj) {
	/*if (numObjects == 0) return 0;
	assert(obj->next);
	if (obj->next == obj) {
		head = 0;
		tail = 0;
		obj->previous = obj->next = 0;
		numObjects--;
		return 1;
	}

	PhysicsObject* current = head;
	do {
		if (current == obj) {
			if (current == head) head = current->next;
			if (current == tail) tail = current->previous;
			current->previous->next = current->next;
			current->next->previous = current->previous; 
			current->previous = current->next = 0;
			numObjects--;
			return 1;
		}
 		current = current->next;

	} while (current != head);*/
	assert(numObjects != 0, "Attempted to remove an object from a node with no contents");
	numObjects--;
	contents.remove(obj);
	return 0;

//debugcheck:
//	if (!head && numObjects != 0) {
//		std::cerr << "Fucking thing isn't clear!..\n";
//	}
//	return 1;
}

void PhysicsController::CollisionNode::clear() {
	numObjects = 0;
}




template <typename T>
PhysicsController::ObjectSpawner<T>::ObjectSpawner(PhysicsController* ctrlr, glm::vec2 p, glm::vec2 dir, float mag) {
	controller = ctrlr;
	position = p;
	exitVelocity = mag * dir;
}

template <typename T>
void PhysicsController::ObjectSpawner<T>::shoot(float timeDelta) {
	controller->addObject(new T(controller, position, OBJECT_SIZE, exitVelocity));
}

template <typename T>
void PhysicsController::ObjectSpawner<T>::update(float timeDelta) {
	if (!keepShooting) return;
	timeSinceLastShot += timeDelta;
	if (timeSinceLastShot > REFRACTORY_TIME) {
		shoot(timeDelta);
		timeSinceLastShot -= REFRACTORY_TIME;
	}
}

template <typename T>
void PhysicsController::ObjectSpawner<T>::start() {
	keepShooting = true;
}

template <typename T>
void PhysicsController::ObjectSpawner<T>::stop() {
	keepShooting = false;
}


PhysicsController::PhysicsController(uint16_t simulationWidth_, uint16_t simulationHeight_) {
	nextID = 0;

	simulationWidth = simulationWidth_;
	simulationHeight = simulationHeight_;

	uint16_t gridWidth = static_cast<uint16_t>(floor(static_cast<float>(simulationWidth) / CELL_SIZE) + 1);
	uint16_t gridHeight = static_cast<uint16_t>(floor(static_cast<float>(simulationHeight) / CELL_SIZE) + 1);

	grid = new CollisionGrid(gridWidth, gridHeight, this);
	pool = new ThreadPool(THREAD_COUNT);

	addSpawnerN({ 75, 75 }, { 1, 0 }, SPAWNER_EXIT_SPEED, 1);
}

PhysicsController::~PhysicsController() {
	for (PhysicsObject* obj : objects) delete obj;
	for (auto spawner : spawners) delete spawner;
	delete grid;
	delete pool;
}

void PhysicsController::addSpawner(glm::vec2 position, glm::vec2 direction, float magnitude) {
	spawners.emplace_back(new ObjectSpawner<PhysicsObject>(this, position + SPAWNER_OFFSET * static_cast<float>(spawners.size()), direction, magnitude));
}

void PhysicsController::addSpawnerN(glm::vec2 p, glm::vec2 dir, float mag, uint8_t n) { 
	for (uint8_t i{ n }; i--;) addSpawner(p, dir, mag); 
}

size_t PhysicsController::getNumObjects() {
	return objects.size();
}

void PhysicsController::addObject(PhysicsObject* obj) {
	objects.push_back(obj);
#ifdef USE_QUEUE
	grid->insert(obj);
#endif
}


void PhysicsController::stopSpawners() {
	for (auto spawner : spawners) spawner->stop();
}
 
void PhysicsController::startSpawners() {
	for (auto spawner : spawners) spawner->start();
}

void PhysicsController::update(float dt) {
	dt = fmin(dt, MAX_TIME_STEP);
	if (objects.size() >= MAX_OBJECTS) { stopSpawners(); }
	for (auto spawner : spawners) {
		spawner->update(dt);
	}
#ifdef USE_QUEUE
	// add all of the objects into the collision queue
	for (auto obj : objects) {
		obj->accelerate(glm::vec2(0, GRAVITATIONAL_FORCE));
		obj->update(dt, simulationWidth, simulationHeight);
		grid->addCollisionsToQueue(obj, dt);
	}

	// go through the queue and run all of the potential collisions
	grid->checkCollisionsQueue(dt);

	// update all objects to the end of the timestep
	for (auto obj : objects) {
		if (obj->infrastepTime != dt) {
			glm::vec2 newPos = obj->position + obj->velocity * (dt - obj->infrastepTime);
			obj->position = newPos;
		}
		// reset infrastepTime for the next frame
		obj->infrastepTime = 0.f;
	}
}
#else

	for (PhysicsObject* obj : objects) {
		obj->accelerate(glm::vec2(0, GRAVITATIONAL_FORCE));
		obj->update(dt, simulationWidth, simulationHeight);
		obj->enforceBoundaries(simulationWidth, simulationHeight);
	}
	handleCollisionsIterations(COLLISION_ITERATIONS);
}

void PhysicsController::handleCollisionsIterations(uint8_t iterations) {
	for (int i{ iterations }; i--;) handleCollisions();
}

void PhysicsController::handleCollisions() {


#ifdef USE_COLLISION_GRID
	grid->clear();
	for (PhysicsObject* obj : objects) {
		assert(grid->insert(obj));
	}

#ifdef USE_THREADS 
	grid->handleCollisionsThreaded(pool);
#else
	grid->handleCollisions();
#endif



#else
	for (PhysicsObject* obj1 : objects) {
		for (PhysicsObject* obj2 : objects) {
			if (obj1 != obj2) {
				grid->checkCollision(obj1, obj2);
			}
		}
	}
#endif
}
#endif



void PhysicsController::displaySimulation() {
	ImGui::SetNextWindowSize({ static_cast<float>(simulationWidth) + IMGUI_FRAME_MARGIN * 2, static_cast<float>(simulationHeight) + IMGUI_FRAME_MARGIN * 2});
	ImGui::SetNextWindowContentSize({ static_cast<float>(simulationWidth), static_cast<float>(simulationHeight) });
	ImGui::Begin("balls", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar);
	
	ImVec2 posWindowOffset = ImGui::GetWindowPos();
	for (PhysicsObject* obj : objects) {
		ImVec2 posBallOffset = { obj->position.x + posWindowOffset.x + IMGUI_FRAME_MARGIN, obj->position.y + posWindowOffset.y + IMGUI_FRAME_MARGIN };
		ImGui::GetWindowDrawList()->AddCircleFilled(posBallOffset, obj->radius, obj->color);
	}
#ifdef DEBUG
	for (int i = 0; i <= simulationWidth / CELL_SIZE; i++) {
		ImVec2 gridLineMin = { posWindowOffset.x + i * CELL_SIZE + IMGUI_FRAME_MARGIN, posWindowOffset.y + IMGUI_FRAME_MARGIN };
		ImVec2 gridLineMax = { posWindowOffset.x + i * CELL_SIZE + IMGUI_FRAME_MARGIN, posWindowOffset.y + simulationHeight + IMGUI_FRAME_MARGIN };
		ImGui::GetWindowDrawList()->AddLine(gridLineMin, gridLineMax, 0xff333333);
	}
	for (int i = 0; i <= simulationHeight / CELL_SIZE; i++) {
		ImVec2 gridLineMin = { posWindowOffset.x + IMGUI_FRAME_MARGIN, posWindowOffset.y + i * CELL_SIZE + IMGUI_FRAME_MARGIN };
		ImVec2 gridLineMax = { posWindowOffset.x + simulationWidth + IMGUI_FRAME_MARGIN, posWindowOffset.y + i * CELL_SIZE + IMGUI_FRAME_MARGIN  };
		ImGui::GetWindowDrawList()->AddLine(gridLineMin, gridLineMax, 0xff333333);
	}

	for (auto node : grid->gridSquares) {
		if (node->count()) {
			uint32_t boxColor = 0xff33ff33;
			ImVec2 nodeBoxMin = { posWindowOffset.x + node->index.x * CELL_SIZE + IMGUI_FRAME_MARGIN,  posWindowOffset.y + node->index.y * CELL_SIZE + IMGUI_FRAME_MARGIN };
			ImVec2 nodeBoxMax = { posWindowOffset.x + node->index.x * CELL_SIZE + CELL_SIZE + IMGUI_FRAME_MARGIN,  posWindowOffset.y + node->index.y * CELL_SIZE + CELL_SIZE + IMGUI_FRAME_MARGIN };
			if (node->count() > 1) boxColor = 0xff33ffff;
			ImGui::GetWindowDrawList()->AddRect(nodeBoxMin, nodeBoxMax, boxColor );
		}
	}
#endif
	ImGui::End();
}
