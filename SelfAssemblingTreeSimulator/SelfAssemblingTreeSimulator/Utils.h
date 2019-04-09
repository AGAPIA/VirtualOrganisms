#ifndef UTILS_H
#define UTILS_H

// Structure mode
#define DIRECTIONAL_MODE 0
#define LEFTRIGHTONLY_MODE 1
#define STRUCTURE_MODE LEFTRIGHTONLY_MODE

// Simulation mode
#define SIMULATE_TREE_COLECTOR 0
#define SIMULATE_PS_MODEL 1
#define SIMULATION_MODE SIMULATE_PS_MODEL

#define DONATE_INTERNAL

#define MIN_MEMBRANE_SIZE 7
#define MAX_MEMBRANE_SIZE 7

#include <unordered_map>
//#include <unordered_set>
#include <string>
#include <assert.h>
#include <atomic>
#include <limits.h>
#include <ostream>

using uint = unsigned int;
#define MAX_ROWS 20 // For emodel use 20!
#define MAX_COLS 20

#define INVALID_POS -1
#define MIN_SCORE 0.0f
#define BOARD_SKIP_CHARACTER '*'
#define SIMULATION_BOARD_HISTORY_SIZE 10

#define INVALID_MAX_OFFSET INT_MIN
#define INVALID_MIN_OFFSET INT_MAX

// TODO: optimize the complexity of this op
#define COMPUTE_SCORE_BY_SIMULATION
#define INVALID_COST_PER_RESOURCE -1.0f

#define EPSILON 0.00001f

enum SourceType
{
	ST_GENERIC,
	ST_PUBLISHER,
	ST_SUBSCRIBER,
};

//#define USE_NODES_SHUFFLING 

struct TablePos
{
	TablePos() : row(INVALID_POS), col(INVALID_POS) {}
	TablePos(int _row, int _col) :row(_row), col(_col) {}
	int row, col;

	bool operator==(const TablePos& other) const {
		return row == other.row && col == other.col;
	}

	bool operator!=(const TablePos& other) const {
		return row != other.row || col != other.col;
	}

	void operator+=(const TablePos& other) {
		row += other.row;
		col += other.col;
	}

	bool operator<(const TablePos& other) const
	{
		if (row < other.row)
			return true;

		if (row > other.row)
			return false;

		if (col < other.col)
			return true;

		return false;
	}

	TablePos operator+(const TablePos& other) const {		
		return TablePos(row + other.row, col + other.col);
	}

	friend std::ostream & operator << (std::ostream &out, const TablePos &srcInfo);
};

int randRange(int min, int max);
bool isCoordinateValid(int row, int col);
bool isCoordinateValid(const TablePos& pos);
float randUniform();

TablePos get2DNormDir(const TablePos& from, const TablePos& to);

bool floatEqual(const float val1, const float val2);

int manhattanDist(const TablePos& p1, const TablePos& p2);
TablePos getRandomTablePos();


struct TablePosHasher
{
	std::size_t operator() (const TablePos& tablePos) const
	{
		return tablePos.row * MAX_COLS + tablePos.col;
	}
};


struct SourceInfo
{
	SourceInfo() : currentPower(0.0f), powerTarget(0.0f){}

	float getPower() const { return currentPower; }
	float getTarget() const { return powerTarget; }

	void setPowerTarget(float value) { powerTarget = value; }
	void setCurrentPower(float value) { currentPower = value; }

	// This function overrides the current power target
	void overridePower(float value)
	{
		currentPower = value;
		powerTarget = value;
	}

#if SIMULATION_MODE==SIMULATE_PS_MODEL
	SourceType sourceType = ST_GENERIC;
	std::string serviceType = "default";

	struct LinkInfo
	{
		float flow; // The capacity used between a source and a destination
		std::vector<TablePos> mirrorNodesUsed; // The nodes (positions) used as mirrors inside VO for this connection
		bool operator==(const LinkInfo& other)
		{
			return flow == other.flow && mirrorNodesUsed == other.mirrorNodesUsed;
		}
	};

	// The set of connected targets. If it is a publisher we can expect multiple sources connected to it (satisfying their request), OUT links
	// If it is a subscriber, it is showing IN links, can be multiple since a subscriber can be attached to multiple publisher since a single one may not satisfy their request
	std::unordered_map<TablePos, LinkInfo, TablePosHasher> m_connectedTo;

	// This shows how much power is used now by connection. 
	// If it is a subscriber, it shows how much we succeeded to use from publishers
	// If it is a publisher, it shows how much we succeeded to give away
	float m_usedPower = 0.0f;

	float getUsedPower() const { return m_usedPower; }
	float getRemainingPower() const { return currentPower - m_usedPower; }

	// Both publishers and subscribers will call this. It will mutate both this and other source info
	void AddConnectionFromPublisherToSubscriber(const TablePos& thisPos, const TablePos& otherPos, SourceInfo& other, const SourceInfo::LinkInfo& linkData);

	// This is called when one side is removed (this pointer), and needs to update other
	void RemoveConnectionTo(const TablePos& thisPos, const TablePos& otherPos, SourceInfo& other);

	void do_sanitycheck_powerUsed();
#endif

	friend std::ostream & operator << (std::ostream &out, const SourceInfo &srcInfo);

private:
	float currentPower;	// This is the actual power of the source
	float powerTarget;	// This is the power target that this source is trying to achieve
};


SourceInfo getRandomSourceInfo();


void trimCommentsAndWhiteSpaces(std::string& str);

struct BufferedTrafficData
{
	BufferedTrafficData(float maxFlowSize) :m_maxFlowSize(maxFlowSize), m_value(0.0f){}

	void add(const float _value, const bool ignoreConstraints = false) 
	{ 
		if (!ignoreConstraints)
			assert(m_value <= m_maxFlowSize);

		m_value += _value;

		if (!ignoreConstraints)
			assert(m_value <= m_maxFlowSize);
	}

	void subtract(const float _value)
	{
		assert(m_value >= 0.0f);
		m_value -= _value;
		assert(m_value >= 0.0f);
	}

	float getCurrentCap() const { return m_value; }
	void reset() { m_value = 0.0f; }


private:
	const float m_maxFlowSize; // MAXIMUM FLOW SIZE SUPPORTED for this buffer
	float m_value; // TODO: atomic / mutex something
};

template <typename T>
T mysgn(T value)
{
	return (T(0.0f) < value) - (value < T(0.0f));
}

#endif
