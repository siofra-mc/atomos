#include "Physics.hpp"

PhysicsController::CollisionGrid::CollisionGrid(uint16_t m, uint16_t n, PhysicsController* ctrlr) : GridContainer<CollisionNode>(m, n, CELL_SIZE), controller(ctrlr) {}

void PhysicsController::CollisionGrid::checkCollision(PhysicsObject* obj1, PhysicsObject* obj2) {
	glm::vec2 distanceVector = obj1->position - obj2->position;
	float dist = glm::length(distanceVector);
	float minDist = obj1->radius + obj2->radius;
	if (dist < minDist) {
		glm::vec2 collisionAxis = distanceVector / dist;
		float delta = minDist - dist;

		glm::vec2 repositionDistance1 = .5f * delta * glm::normalize(collisionAxis);
		glm::vec2 repositionDistance2 = -.5f * delta * glm::normalize(collisionAxis);
		float factorMass1 = 2 * obj2->mass / (obj1->mass + obj2->mass);
		float factorMass2 = 2 * obj1->mass / (obj1->mass + obj2->mass);
		glm::vec2 positionDiffVector = obj1->position - obj2->position;
		glm::vec2 velocityDiffVector = obj1->velocity - obj2->velocity;

		glm::vec2 velocityAdjustment1 = factorMass1 * glm::dot(velocityDiffVector, positionDiffVector) / glm::dot(positionDiffVector, positionDiffVector) * positionDiffVector;
		glm::vec2 velocityAdjustment2 = factorMass2 * glm::dot(-velocityDiffVector, -positionDiffVector) / glm::dot(positionDiffVector, positionDiffVector) * -positionDiffVector;

		obj1->position += repositionDistance1;
		obj2->position += repositionDistance2;

		obj1->velocity -= velocityAdjustment1 * ELASTICITY;
		obj2->velocity -= velocityAdjustment2 * ELASTICITY;

		obj1->enforceBoundaries(800, 700);
		obj2->enforceBoundaries(800, 700);
	}
}

// finds the initial solution to the intersection of two objects (ie. when two circles intersect at exactly one point)
int PhysicsController::CollisionGrid::interpolateInitalIntersection(PhysicsObject* obj1, PhysicsObject* obj2) {
	// perform quadratic equation to find event time
	glm::vec2 distanceDifference = obj1->position - obj2->position;
	glm::vec2 velocityDifference = obj1->velocity - obj2->velocity;
	float minDistance = obj1->radius + obj2->radius;

	float distanceDifferenceInnerProduct = glm::dot(distanceDifference, distanceDifference);
	float velocityDifferenceInnerProduct = glm::dot(velocityDifference, velocityDifference);
	float bterm = glm::dot(distanceDifference, velocityDifference) / velocityDifferenceInnerProduct;
	float determinate = bterm * bterm - (distanceDifferenceInnerProduct - minDistance * minDistance) / velocityDifferenceInnerProduct;

	// 1 solution if == 0, 2 solutions if >0, no real solutions if <0
	if (determinate >= 0) {
		// we only care about the earliest collision time
		float eventTime = -bterm - sqrt(determinate);
		return (eventTime >= 0);
	}
}

//void PhysicsController::CollisionGrid::checkCellCollisions(CollisionNode* cell1, CollisionNode* cell2) {
//	for (int i = 0; i < cell1->count(); i++) {
//		PhysicsObject* obj1 = cell1->getObject(i);
//		for (int j = 0; j < cell2->count(); j++) {
//			PhysicsObject* obj2 = cell1->getObject(j);
//			if (obj1 != obj2) {
//#ifdef USE_QUEUE
//				checkCollisionToQueue(obj1, obj2);
//#else
//				checkCollision(obj1, obj2);
//#endif
//			}
//		}
//	}
//}

//void PhysicsController::CollisionGrid::handleCollisions(int widthLow = 1, int widthHigh = -1) {
//	//work around since I can't put member variables in default parameters
//	if (widthHigh == -1 || widthHigh >= width) widthHigh = width - 1;
//
//
//	for (int j = 1; j < height - 1; j++) {
//		for (int i = widthLow; i < widthHigh; i++) {
//			CollisionNode* currentNode = getCell(i, j);
//			if (currentNode->count() == 0) continue;
//			for (int dj = -1; dj <= 1; dj++) {
//				for (int di = -1; di <= 1; di++) {
//					CollisionNode* adjacentNode = getCell(i + di, j + dj);
//					if (adjacentNode->count() == 0) continue;
//					checkCellCollisions(currentNode, adjacentNode);
//				}
//			}
//		}
//	}
//}

#ifndef USE_QUEUE

void PhysicsController::CollisionGrid::handleCollisionsThreaded(ThreadPool* pool) {
	float step = static_cast<float>(width) / static_cast<float>(THREAD_COUNT);
	for (int i = 0; i < THREAD_COUNT; i++) {
		int widthRangeLow = static_cast<int>(step * i) + 1;
		int widthRangeHigh = static_cast<int>(step * (i + 1)) + 1;
		pool->addTask(std::bind(&CollisionGrid::handleCollisions, this, widthRangeLow, widthRangeHigh));
	}
}

#endif

void PhysicsController::CollisionGrid::addCollisionsToQueue(PhysicsObject* object, float dt) {
	float occuranceTime;

	float eventTime = dt;
	bool eventOccured = false;
	Direction eventDirection = NONE;
	CollisionEvent::CollisionType type = CollisionEvent::ERROR;
	PhysicsObject* predicateObject = 0;

	CollisionNode* currentNode = getCellFromObjectID(object->id);
	glm::vec2 newPosition = object->position + object->velocity * (dt - object->infrastepTime);

	assert(currentNode != nullptr);


	// check for boundary enforcements
	if (newPosition.x < object->radius) {
		occuranceTime = object->infrastepTime + abs(object->radius - object->position.x) / abs(object->velocity.x);
		if (object->position.x < object->radius) occuranceTime = 0;
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = LEFT;
			type = CollisionEvent::BOUNDARY_ENFORCEMENT;
		}
	}
	if (newPosition.y < object->radius) {
		occuranceTime = object->infrastepTime + abs(object->radius - object->position.y) / abs(object->velocity.y);
		if (object->position.y < object->radius) occuranceTime = 0;
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = UP;
			type = CollisionEvent::BOUNDARY_ENFORCEMENT;
		}
	}
	if (newPosition.x > controller->simulationWidth - object->radius) {
		occuranceTime = object->infrastepTime + abs(object->position.x - (controller->simulationWidth - object->radius)) / abs(object->velocity.x);
		if (object->position.x > controller->simulationWidth - object->radius) occuranceTime = 0;
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = RIGHT;
			type = CollisionEvent::BOUNDARY_ENFORCEMENT;
		}
	}
	if (newPosition.y > controller->simulationHeight - object->radius) {
		occuranceTime = object->infrastepTime + abs(object->position.y - (controller->simulationHeight - object->radius)) / abs(object->velocity.y);
		if (object->position.y > controller->simulationHeight - object->radius) occuranceTime = 0;
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = DOWN;
			type = CollisionEvent::BOUNDARY_ENFORCEMENT;
		}
	}

	// check for cell changes
	if (newPosition.x <= currentNode->minimumBound.x) {
		occuranceTime = object->infrastepTime + abs(currentNode->minimumBound.x - object->position.x) / abs(object->velocity.x);
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = LEFT;
			type = CollisionEvent::CELL_CHANGE;
		}
	}
	if (newPosition.y <= currentNode->minimumBound.y) {
		occuranceTime = object->infrastepTime + abs(currentNode->minimumBound.y - object->position.y) / abs(object->velocity.y);
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = UP;
			type = CollisionEvent::CELL_CHANGE;
		}
	}
	if (newPosition.x > currentNode->maximumBound.x) {
		occuranceTime = object->infrastepTime + abs(currentNode->maximumBound.x - object->position.x) / abs(object->velocity.x);
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = RIGHT;
			type = CollisionEvent::CELL_CHANGE;
		}
	}
	if (newPosition.y > currentNode->maximumBound.y) {
		occuranceTime = object->infrastepTime + abs(currentNode->maximumBound.y - object->position.y) / abs(object->velocity.y);
		if (occuranceTime < eventTime) {
			eventOccured = true;
			eventTime = occuranceTime;
			eventDirection = DOWN;
			type = CollisionEvent::CELL_CHANGE;
		}
	}

	// check for object collisions
	for (int dj = -1; dj <= 1; dj++) {
		for (int di = -1; di <= 1; di++) {
			int xAdjacentNodeIndex = currentNode->index.x + di;
			int yAdjacentNodeIndex = currentNode->index.y + dj;
			if (xAdjacentNodeIndex < 0 || yAdjacentNodeIndex < 0 || xAdjacentNodeIndex >= width || yAdjacentNodeIndex >= height) continue;
			CollisionNode* adjacentNode = getCellFromIndex(xAdjacentNodeIndex, yAdjacentNodeIndex);
			if (adjacentNode->count() == 0) continue;

			//for (int i = 0; i < adjacentNode->count(); i++) {
			//	PhysicsObject* obj2 = adjacentNode->getObject(i);
			for (PhysicsObject* obj2 : adjacentNode->contents) {
				if (!obj2 || object == obj2) continue;

				// perform quadratic equation to find event time
				glm::vec2 distanceDifference = object->position - obj2->position;
				glm::vec2 velocityDifference = object->velocity - obj2->velocity;
				float minDistance = object->radius + obj2->radius;


				if (glm::length(distanceDifference) < minDistance) {
					eventOccured = true;
					type = CollisionEvent::BALL_OVERLAP;
					eventTime = object->infrastepTime;
					eventDirection = NONE;
					predicateObject = obj2;
					break;
				}


				float distanceDifferenceInnerProduct = glm::dot(distanceDifference, distanceDifference);
				float velocityDifferenceInnerProduct = glm::dot(velocityDifference, velocityDifference);
				float bterm = glm::dot(distanceDifference, velocityDifference) / velocityDifferenceInnerProduct;
				float determinate = (bterm * bterm) - ((distanceDifferenceInnerProduct - minDistance * minDistance) / velocityDifferenceInnerProduct);

				// 1 solution if == 0, 2 solutions if >0, no real solutions if <0
				if (determinate >= 0) {
					// we only care about the earliest collision time
					occuranceTime = object->infrastepTime - bterm - sqrt(determinate);
					if (occuranceTime < eventTime && occuranceTime >= 0) {
						std::cout << "Collision\n\tTime: " << occuranceTime << std::endl;
						eventOccured = true;
						type = CollisionEvent::BALL_BALL;
						eventTime = occuranceTime;
						eventDirection = NONE;
						predicateObject = obj2;
					}
				}
			}
		}
	}

	if (eventOccured) {
		CollisionEvent newEvent = { type, eventTime, object, eventDirection, predicateObject };
		eventQueue.push(newEvent);
	}
}

uint32_t queueCount = 0;
void PhysicsController::CollisionGrid::checkCollisionsQueue(float dt) {
	queueCount = 0;
	CollisionEvent nextCollision;
	CollisionNode* ppp = 0;
	bool skipReentry;
	while (!eventQueue.empty()) {
		skipReentry = false;
		queueCount++;
		nextCollision = eventQueue.top(); eventQueue.pop();
		//std::cerr << "Processing Collision:\n\tCollision Type: ";
		switch (nextCollision.type) {
		case CollisionEvent::CELL_CHANGE:
			std::cerr << "Cell Change\n\tDirection: " << nextCollision.eventDirection << "\n\tObject ID: " << nextCollision.subjectObject->id << '\n';
			/*		if (skipReentry) skipReentry = false; continue; */
			if (nextCollision.eventTime - nextCollision.subjectObject->infrastepTime == 0) skipReentry = true;

			remove(nextCollision.subjectObject);

			nextCollision.subjectObject->position += nextCollision.subjectObject->velocity * static_cast<float>(nextCollision.eventTime - nextCollision.subjectObject->infrastepTime);
			if (nextCollision.eventDirection == LEFT || nextCollision.eventDirection == UP) nextCollision.subjectObject->nudge();

			insert(nextCollision.subjectObject);
			break;


		case CollisionEvent::BOUNDARY_ENFORCEMENT:
			//std::cerr << "Boundary Enforcement";
			remove(nextCollision.subjectObject);
			if (nextCollision.eventTime == 0) {
				switch (nextCollision.eventDirection) {
				case UP:
					nextCollision.subjectObject->position.y = nextCollision.subjectObject->radius;
					break;
				case DOWN:
					nextCollision.subjectObject->position.y = controller->simulationHeight - nextCollision.subjectObject->radius;
					break;
				case LEFT:
					nextCollision.subjectObject->position.x = nextCollision.subjectObject->radius;
					break;
				case RIGHT:
					nextCollision.subjectObject->position.x = controller->simulationWidth - nextCollision.subjectObject->radius;
					break;
				}
			}
			else {
				nextCollision.subjectObject->position += nextCollision.subjectObject->velocity * static_cast<float>(nextCollision.eventTime - nextCollision.subjectObject->infrastepTime);
			}
			if (nextCollision.eventDirection == UP || nextCollision.eventDirection == DOWN) nextCollision.subjectObject->velocity.y *= -ELASTICITY;
			if (nextCollision.eventDirection == LEFT || nextCollision.eventDirection == RIGHT) nextCollision.subjectObject->velocity.x *= -ELASTICITY;
			//nextCollision.subjectObject->velocity *= ELASTICITY;

			insert(nextCollision.subjectObject);

			break;


		case CollisionEvent::BALL_BALL:
			//std::cerr << "Ball to Ball";
			remove(nextCollision.subjectObject);
			remove(nextCollision.predicateObject);

			// This is scoped because otherwise the compiler returns a bunch of warnings
			{
				// move the balls up to the collision point
				glm::vec2 newSubjectPosition = nextCollision.subjectObject->position + nextCollision.subjectObject->velocity * (nextCollision.eventTime - nextCollision.subjectObject->infrastepTime);
				glm::vec2 newPredicatePosition = nextCollision.predicateObject->position + nextCollision.predicateObject->velocity * (nextCollision.eventTime - nextCollision.predicateObject->infrastepTime);

				nextCollision.subjectObject->position = newSubjectPosition;
				nextCollision.predicateObject->position = newPredicatePosition;

				float factorMass1 = 2 * nextCollision.predicateObject->mass / (nextCollision.subjectObject->mass + nextCollision.predicateObject->mass);
				float factorMass2 = 2 * nextCollision.subjectObject->mass / (nextCollision.subjectObject->mass + nextCollision.predicateObject->mass);
				glm::vec2 positionDiffVector = nextCollision.subjectObject->position - nextCollision.predicateObject->position;
				glm::vec2 velocityDiffVector = nextCollision.subjectObject->velocity - nextCollision.predicateObject->velocity;

				glm::vec2 velocityAdjustment1 = factorMass1 * glm::dot(velocityDiffVector, positionDiffVector) / glm::dot(positionDiffVector, positionDiffVector) * positionDiffVector;
				glm::vec2 velocityAdjustment2 = factorMass2 * glm::dot(-velocityDiffVector, -positionDiffVector) / glm::dot(positionDiffVector, positionDiffVector) * -positionDiffVector;

				nextCollision.subjectObject->velocity -= velocityAdjustment1 * ELASTICITY;
				nextCollision.predicateObject->velocity -= velocityAdjustment2 * ELASTICITY;

				nextCollision.predicateObject->infrastepTime = nextCollision.eventTime;
			}
			insert(nextCollision.subjectObject);
			insert(nextCollision.predicateObject);
			break;

		case CollisionEvent::BALL_OVERLAP:
		{
#define TEST  

#ifdef TEST
			remove(nextCollision.subjectObject);
			remove(nextCollision.predicateObject);

#else

			CollisionNode* obj1Cell = getCellFromPosition(nextCollision.subjectObject->position);
			CollisionNode* obj2Cell = getCellFromPosition(nextCollision.predicateObject->position);

			assert(obj1Cell && obj2Cell);
			obj1Cell->remove(nextCollision.subjectObject);
			obj2Cell->remove(nextCollision.predicateObject);
#endif

			glm::vec2 overlapVector = nextCollision.subjectObject->position - nextCollision.predicateObject->position;
			glm::vec2 collisionAxis = glm::normalize(overlapVector);
			float overlapDistance = (nextCollision.subjectObject->radius + nextCollision.predicateObject->radius) - glm::length(overlapVector);

			nextCollision.subjectObject->position += overlapDistance / 2;
			nextCollision.predicateObject->position -= overlapDistance / 2;

#ifdef TEST
			insert(nextCollision.subjectObject);
			insert(nextCollision.predicateObject);

#else

			obj1Cell = getCellFromPosition(nextCollision.subjectObject->position);
			obj2Cell = getCellFromPosition(nextCollision.predicateObject->position);

			obj1Cell->insert(nextCollision.subjectObject); j
				obj2Cell->insert(nextCollision.predicateObject);

#endif
			skipReentry = true;
		}
		break;
		default:
			std::cerr << "\n\nBad Collision Event Detected\n\tSubject Object ID: " << nextCollision.subjectObject->id
				<< "\n\tTime of Event: " << nextCollision.eventTime << std::endl;
			exit(1000);
			break;
		}
		//std::cerr << "\n";
		nextCollision.subjectObject->infrastepTime = nextCollision.eventTime;
		if (!skipReentry && nextCollision.subjectObject->infrastepTime < dt) {
			addCollisionsToQueue(nextCollision.subjectObject, nextCollision.eventTime);
		}
	}
}
