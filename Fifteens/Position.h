#ifndef POSITION
#define POSITION

#include <array>
#include <set>
#include <queue>
#include <string>
#include <list>
#include <cassert>
#include <cmath>
#include <cstdlib>

#define FieldSize 4

class Position {
public:
	std::array<int, FieldSize*FieldSize> numerics;
	int distanceToThisPosition;
	int estimation;
public:
	const Position* previous;
	char wayOfCreating;
public:
	Position(const std::array<int, FieldSize*FieldSize>& Numerics);
	Position(const Position& position_, const char& wayOfCreating_);

	bool is_solvable() const;
	bool is_solved() const;
	int nullPos() const;
	std::string bfs() const;
	std::string AStar() const;
	bool operator<(const Position& otherPosition) const;//для хранения в set

	bool canMoveDown() const;
	bool canMoveUp() const;
	bool canMoveLeft() const;
	bool canMoveRight() const;

	void moveDown();
	void moveUp();
	void moveLeft();
	void moveRight();

	int estimationOfDistance() const;
};



#endif