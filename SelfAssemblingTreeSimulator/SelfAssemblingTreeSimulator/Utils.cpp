#include "Utils.h"
#include <algorithm>
#include "Cell.h"
#include <assert.h>

extern int g_minPowerForWirelessSource;
extern	int g_maxPowerForWirelessSource;

extern int g_maxPublisher_inflow;		// max publisher capacity
extern int g_maxPublisher_outflow;		// max consumer capacity
extern float g_probability_to_spawn_publisher;

int randRange(int min, int max)
{
	return min + (rand() % static_cast<int>(max - min + 1));
}

bool floatEqual(const float val1, const float val2)
{
	return std::fabsf(val1 - val2) < EPSILON;
}

float randUniform()
{
	return rand() / (float)RAND_MAX;
}

TablePos getRandomTablePos()
{
	TablePos pos;
	pos.row = randRange(0, MAX_ROWS - 1);
	pos.col = randRange(0, MAX_COLS - 1);
	return pos;
}

SourceInfo getRandomSourceInfo()
{
	SourceInfo s;

#if SIMULATION_MODE == SIMULATE_TREE_COLECTOR
	s.overridePower((float)randRange(g_minPowerForWirelessSource, g_maxPowerForWirelessSource));
#elif SIMULATION_MODE == SIMULATE_PS_MODEL
	// Decide which one of publisher or subscriber should we spawn
	const float randVal = randUniform();
	if (randVal < g_probability_to_spawn_publisher)
	{
		s.sourceType = ST_PUBLISHER;
		s.overridePower((float)randRange(std::min(1, g_maxPublisher_inflow / 10), g_maxPublisher_inflow));
	}
	else
	{
		s.sourceType = ST_SUBSCRIBER;
		s.overridePower((float)randRange(std::min(1, g_maxPublisher_outflow / 10), g_maxPublisher_outflow));
	}
#endif

	return s;
}

bool isCoordinateValid(int row, int col)
{
	return (row >= 0 && col >= 0 && row < MAX_ROWS && col < MAX_COLS);
}

bool isCoordinateValid(const TablePos& pos)
{
	return isCoordinateValid(pos.row, pos.col);
}

TablePos get2DNormDir(const TablePos& from, const TablePos& to)
{
	int rowDir = to.row - from.row;
	rowDir = (rowDir < 0 ? -1 : 1) * std::min(std::abs(rowDir), 1);

	int colDir = to.col - from.col;
	colDir = (colDir < 0 ? -1 : 1) * std::min(std::abs(colDir), 1);

	return TablePos(rowDir, colDir);
}

int manhattanDist(const TablePos& p1, const TablePos& p2)
{
	// Remapped to avoid 0 distances !!!
	return 1 + std::abs(p1.row - p2.row) + std::abs(p1.col - p2.col);
}

void trimCommentsAndWhiteSpaces(std::string& str)
{
	static char* whitespaces = " \t";
	static char* comments = "//";

	// Eliminate comments
	const int commentPos = (int)str.find(comments);
	if (commentPos != std::string::npos)
	{
		str.erase(str.begin() + commentPos, str.end());
	}
	else
	{
		int a = 3;
		a++;
	}

	// Left side
	const auto firstNonWhiteSpace = str.find_first_not_of(whitespaces);
	if (firstNonWhiteSpace == std::string::npos)
	{
		str.clear();
		return;
	}
	str.erase(0, firstNonWhiteSpace);

	// Right side
	const auto lastNonWhiteSpace = str.find_last_not_of(whitespaces);
	if (lastNonWhiteSpace == std::string::npos)
	{
		str.clear();
		return;
	}

	str.erase(str.begin() + lastNonWhiteSpace + 1, str.end());
}

void SourceInfo::AddConnectionFromPublisherToSubscriber(const TablePos& thisPos, const TablePos& otherPos, SourceInfo& other, const SourceInfo::LinkInfo& linkData)
{
	assert(other.sourceType == ST_PUBLISHER && "Please call this only for publisher. Consumer logic will be treated from publishers one automatically");
	assert((this->getRemainingPower() > 0 && other.getRemainingPower() > 0) && "You are trying two connect two nodes but at least one of them has 0 remaining capacity");

	// Check if both sides don't have this connection
	auto itInOther = other.m_connectedTo.find(thisPos);
	auto itInThis = this->m_connectedTo.find(otherPos);
	assert(itInOther == other.m_connectedTo.end() && "This connection already exists in other");
	assert(itInThis == this->m_connectedTo.end() && "This connection already exists in this");

	this->m_connectedTo.insert(std::make_pair(otherPos, linkData));
	this->m_usedPower += linkData.flow;

	other.m_connectedTo.insert(std::make_pair(thisPos, linkData));
	other.m_usedPower += linkData.flow;

	this->do_sanitycheck_powerUsed();
	other.do_sanitycheck_powerUsed();
}

void SourceInfo::RemoveConnectionTo(const TablePos& thisPos, const TablePos& otherPos, SourceInfo& other)
{
	// Remove from "other" connections to thisPos
	auto itInOther = other.m_connectedTo.find(thisPos);
	auto itInThis  = this->m_connectedTo.find(otherPos);
	
	assert(itInOther != other.m_connectedTo.end() && "Couldn't find this connection in other!");
	assert(itInThis != this->m_connectedTo.end() && "Couldn't find this connection in this!");

	const LinkInfo& linkInfo = itInThis->second;
	assert(itInOther->second == linkInfo); // links must match in both directions

	// Subtract the power used from the link
	this->m_usedPower	-= linkInfo.flow;
	other.m_usedPower	-= linkInfo.flow;

	// Just sanity checks
	this->do_sanitycheck_powerUsed();
	other.do_sanitycheck_powerUsed();
}

void SourceInfo::do_sanitycheck_powerUsed()
{
	float sum = 0.0f;
	for (auto& it : m_connectedTo)
	{
		sum += it.second.flow;
	}

	assert((fabs(this->m_usedPower - sum) < EPSILON) && ("Incorrect flow used variable when compared with connected sources!"));

	assert(this->m_usedPower > 0 && this->m_usedPower <= this->getPower());
}


std::ostream & operator << (std::ostream &out, const TablePos &srcInfo)
{
	out << "(" << srcInfo.row << "," << srcInfo.col << ")";
	return out;
}

std::ostream & operator << (std::ostream &out, const SourceInfo &srcInfo)
{
#if SIMULATION_MODE==SIMULATE_PS_MODEL
	out << "Total cap: " << srcInfo.getPower() << " used cap " << srcInfo.getUsedPower() << " remain cap " << srcInfo.getRemainingPower();
	out << std::endl << "Connections:" << std::endl;
	for (const auto& it : srcInfo.m_connectedTo)
	{
		out << "to publisher " << it.first << ". flow " << it.second.flow << " Mirror nodes used: ";
		for (const TablePos& nodePos : it.second.mirrorNodesUsed)
		{
			out << nodePos;
		}
	}
#else
	out << "Power: " << srcInfo.getPower();
#endif
	return out;
}

