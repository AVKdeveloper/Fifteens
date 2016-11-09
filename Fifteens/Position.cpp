#include "Position.h"

Position::Position(const std::array<int, FieldSize*FieldSize>& Numerics) {
	for (int i = 0; i < FieldSize*FieldSize; ++i) {
		numerics[i] = Numerics[i];
	}
	previous = nullptr;
	distanceToThisPosition = 0;
	estimation = estimationOfDistance();
}

Position::Position(const Position& position_, const char& wayOfCreating_) {
	numerics = position_.numerics;
	previous = &position_;
	distanceToThisPosition = position_.distanceToThisPosition + 1;
	wayOfCreating = wayOfCreating_;
	switch (wayOfCreating) {
		case 'D':
			moveDown();
			break; 
		case 'U':
			moveUp();
			break;
		case 'L':
			moveLeft();
			break;
		case 'R':
			moveRight();
			break;
	default:
		assert( false );
	}
	estimation = estimationOfDistance();
}

std::string Position::bfs() const {
	std::queue<Position*> Q;
	std::list<Position> L; // ������ ��� �������� ��������������� �������
	L.push_back(*this);
	Q.push(&(*L.begin()));
	std::set<Position> S;
	S.insert(*this);
	std::string str = "";
	while (!Q.empty())
	{
		Position & currentPos = *Q.front();
		Q.pop();
		if (currentPos.is_solved()) {
			int resultDistance = currentPos.distanceToThisPosition;
			Position tempPos(currentPos);
			while (tempPos.previous != nullptr) {
				str = str + tempPos.wayOfCreating;
				tempPos = (*tempPos.previous);
			}
			std::reverse(str.begin(), str.end());
			assert(resultDistance == str.size());
			break;
		}
		else {
			if (currentPos.canMoveLeft()) {// ����� �� ����� ����� 
				Position newPosition(currentPos, 'L');
				if (S.find(newPosition) == S.end()) {
					L.push_back(newPosition);
					S.insert(newPosition);
					Q.push(&(*(L.rbegin())));
				}
			}
			if (currentPos.canMoveRight()) {// ����� �� ����� �������
				Position newPosition(currentPos, 'R');
				if (S.find(newPosition) == S.end()) {
					L.push_back(newPosition);
					S.insert(newPosition);
					Q.push(&(*(L.rbegin())));
				}
			}
			if (currentPos.canMoveUp()) {// ����� �� ����� �����
				Position newPosition(currentPos, 'U');
				if (S.find(newPosition) == S.end()) {
					L.push_back(newPosition);
					S.insert(newPosition);
					Q.push(&(*(L.rbegin())));
				}
			}
			if (currentPos.canMoveDown()) {// ����� �� �� ����� ����
				Position newPosition(currentPos, 'D');
				if (S.find(newPosition) == S.end()) {
					L.push_back(newPosition);
					S.insert(newPosition);
					Q.push(&(*(L.rbegin())));
				}
			}
		}
	}
	return str;
}

std::string Position::AStar() const {
	auto distanceComparator = [](const Position* ptrFirstPosition, const Position* ptrSecondPosition) {
		return ptrFirstPosition->distanceToThisPosition + ptrFirstPosition->estimationOfDistance() >
			ptrSecondPosition->distanceToThisPosition + ptrSecondPosition->estimationOfDistance();
	};
	std::priority_queue<Position*, std::vector<Position*>, decltype(distanceComparator)> queue_of_ptr_positions(distanceComparator);
	std::list<Position> list_of_generated_positions; // ������ ��� �������� ��������������� �������
	list_of_generated_positions.push_back(*this);
	queue_of_ptr_positions.push(&(*list_of_generated_positions.begin()));
	std::set<Position> set_of_visited_positions; // ���������� �������
	std::string str = "";
	while (!queue_of_ptr_positions.empty())
	{
		Position & currentPos = *queue_of_ptr_positions.top();
		queue_of_ptr_positions.pop();
		if (set_of_visited_positions.find(currentPos) != set_of_visited_positions.end()) { // ���� ������� ���� ��������
			continue;
		}
		if (currentPos.is_solved()) {
			int resultDistance = currentPos.distanceToThisPosition;
			Position tempPos(currentPos);
			while (tempPos.previous != nullptr) {
				str = str + tempPos.wayOfCreating;
				tempPos = (*tempPos.previous);
			}
			std::reverse(str.begin(), str.end());
			assert(resultDistance == str.size());
			break;
		}
		else {
			if (currentPos.canMoveLeft()) {// ����� �� ����� ����� 
				Position newPosition(currentPos, 'L');
				if (set_of_visited_positions.find(newPosition) == set_of_visited_positions.end()) {
					list_of_generated_positions.push_back(newPosition);
					queue_of_ptr_positions.push(&(*(list_of_generated_positions.rbegin())));
				}
			}
			if (currentPos.canMoveRight()) {// ����� �� ����� �������
				Position newPosition(currentPos, 'R');
				if (set_of_visited_positions.find(newPosition) == set_of_visited_positions.end()) {
					list_of_generated_positions.push_back(newPosition);
					queue_of_ptr_positions.push(&(*(list_of_generated_positions.rbegin())));
				}
			}
			if (currentPos.canMoveUp()) {// ����� �� ����� �����
				Position newPosition(currentPos, 'U');
				if (set_of_visited_positions.find(newPosition) == set_of_visited_positions.end()) {
					list_of_generated_positions.push_back(newPosition);
					queue_of_ptr_positions.push(&(*(list_of_generated_positions.rbegin())));
				}
			}
			if (currentPos.canMoveDown()) {// ����� �� �� ����� ����
				Position newPosition(currentPos, 'D');
				if (set_of_visited_positions.find(newPosition) == set_of_visited_positions.end()) {
					list_of_generated_positions.push_back(newPosition);
					queue_of_ptr_positions.push(&(*(list_of_generated_positions.rbegin())));
				}
			}
		}
		set_of_visited_positions.insert(currentPos);
	}
	return str;
}

bool Position::is_solvable() const {
	int quantityReplaces = 0;
	for (int i = 0; i < FieldSize*FieldSize; ++i)
		for (int j = i + 1; j < FieldSize*FieldSize; ++j)
			if (numerics[i] > numerics[j])
				quantityReplaces++;
	quantityReplaces = quantityReplaces + (FieldSize*FieldSize - 1 - this->nullPos());
	if (quantityReplaces % 2 == 0)
		return true;
	else
		return false;
}

bool Position::is_solved() const {
	bool solved = true;
	if (numerics[FieldSize*FieldSize - 1] != 0) {
		solved = false;
	}
	else {
		for (int i = 0; i < FieldSize*FieldSize - 2; ++i) {
			if (numerics[i] != i + 1) {
				solved = false;
				break;
			}
		}
	}
	return solved;
}

int Position::nullPos() const {
	int position = 0;
	for (int i = 0; i < FieldSize*FieldSize; ++i) {
		if (numerics[i] == 0)
			position = i;
	}
	return position;
}

bool Position::operator<(const Position& otherPosition) const {
	bool less = false;
	for (int i = 0; i < FieldSize*FieldSize; ++i) {
		if (numerics[i] < otherPosition.numerics[i]) {
			less = true;
			break;
		}
		else {
			if (numerics[i] > otherPosition.numerics[i]) {
				break;
			}
		}
	}
	return less;
}

bool Position::canMoveDown() const {
	return nullPos() / FieldSize < FieldSize - 1;
}

bool Position::canMoveUp() const {
	return nullPos() / FieldSize > 0;
}

bool Position::canMoveLeft() const {
	return nullPos() % FieldSize > 0;
}

bool Position::canMoveRight() const {
	return nullPos() % FieldSize < FieldSize - 1;
}

void Position::moveDown() {
	auto nullPosition = nullPos();
	auto tmp = numerics[nullPosition];
	numerics[nullPosition] = numerics[nullPosition + FieldSize];
	numerics[nullPosition + FieldSize] = tmp;
}

void Position::moveUp() {
	auto nullPosition = nullPos();
	auto tmp = numerics[nullPosition];
	numerics[nullPosition] = numerics[nullPosition - FieldSize];
	numerics[nullPosition - FieldSize] = tmp;
}

void Position::moveLeft() {
	auto nullPosition = nullPos();
	auto tmp = numerics[nullPosition];
	numerics[nullPosition] = numerics[nullPosition - 1];
	numerics[nullPosition - 1] = tmp;
}

void Position::moveRight() {
	auto nullPosition = nullPos();
	auto tmp = numerics[nullPosition];
	numerics[nullPosition] = numerics[nullPosition + 1];
	numerics[nullPosition + 1] = tmp;
}

int Position::estimationOfDistance() const {
	int result = 0;
	for (int i = 0; i < FieldSize*FieldSize; ++i) {
		if (numerics[i] != 0) {
			result += std::abs((numerics[i]-1) / FieldSize - i / FieldSize) + std::abs((numerics[i]-1) % FieldSize - i % FieldSize);
		}
		else {
			result += 2*FieldSize - 2 - (nullPos() / FieldSize + nullPos() % FieldSize);
		}
	}
	return result;
}