#include "BoardObject.h"
#include "Utils.h"
#include <unordered_map>
#include <assert.h>
#include "ExprGenerator.h"
#include <algorithm>
#include <stack>
#include <algorithm>
#include <set>

#if SIMULATION_MODE==SIMULATE_PS_MODEL
BoardObject::BoardPublisherSubscriberManager::BoardPublisherSubscriberManager(BoardObject* _parent)
{
	parent = _parent;

	reset();
}

void BoardObject::BoardPublisherSubscriberManager::reset()
{
	m_publisherPosToMirrorNodes.clear();
	m_mirrorNodesCollection.clear();

	m_publishersCollection.clear();
	m_subscribersCollection.clear();
}

// Does mirroring support for publisher subscriber model
// Connects publishers and subscriber using a greedy approach
void BoardObject::BoardPublisherSubscriberManager::solvePublishersSubscribersConnections(std::ostream* outStream)
{
	// TODO


	do_sanityChecks();
}

void BoardObject::BoardPublisherSubscriberManager::onItemAdd(const TablePos& pos, const SourceInfo& srcInfo)
{
	assert(srcInfo.sourceType != ST_GENERIC);
	auto& targetCollection = srcInfo.sourceType == ST_SUBSCRIBER ? m_subscribersCollection : m_publishersCollection;
	assert(targetCollection.find(pos) == targetCollection.end() && "This node already exists !");
	targetCollection.insert(pos);

	do_sanityChecks();

	solvePublishersSubscribersConnections(nullptr);
}

void BoardObject::BoardPublisherSubscriberManager::onItemRemoved(const TablePos& pos, const bool removeAll)
{
	if (removeAll)
	{
		reset();
	}
	else
	{
		SourceInfo& srcInfo = parent->m_posToSourceMap[pos];

		// Remove the right collection
		assert(srcInfo.sourceType != ST_GENERIC);
		auto& targetCollection = srcInfo.sourceType == ST_SUBSCRIBER ? m_subscribersCollection : m_publishersCollection;
		assert(targetCollection.find(pos) == targetCollection.end() && "This node already exists !");
		targetCollection.insert(pos);

		// Iterate through all connections for this and remove them
		for (const auto& it : srcInfo.m_connectedTo)
		{
			const TablePos& otherPos = it.first;
			SourceInfo& otherSrcInfo = parent->m_posToSourceMap[otherPos];
			const SourceInfo::LinkInfo& linkInfo = it.second;

			// Decrease ref count for mirrors used along the link
			for (const TablePos& mirrorPos : linkInfo.mirrorNodesUsed)
			{
				MirrorNodeInfo& mirrorNodeInfo = const_cast<MirrorNodeInfo&>(*m_mirrorNodesCollection.find(MirrorNodeInfo(mirrorPos)));
				mirrorNodeInfo.removeSubscriber(pos);
				assert(mirrorNodeInfo.getRefCount() >= 0);

				// This mirror node is not used anymore so the parent publisher must remove it from being used
				const TablePos& publisherPos = srcInfo.sourceType == ST_PUBLISHER ? pos : otherPos;
				if (mirrorNodeInfo.getRefCount() == 0)
				{
					m_publisherPosToMirrorNodes[publisherPos].erase(mirrorPos);
					m_mirrorNodesCollection.erase(mirrorNodeInfo);
				}
			}

			// Remove the connection physically
			srcInfo.RemoveConnectionTo(pos, otherPos, otherSrcInfo);
		}

		do_sanityChecks();

		solvePublishersSubscribersConnections(nullptr);
	}
}

void BoardObject::BoardPublisherSubscriberManager::do_sanityChecks()
{
	// First check:----- are all sources inside m_subscribersCollection and m_publishersCollection ?
	{
		const auto& allSourcesInVO = parent->m_posToSourceMap;
		assert(allSourcesInVO.size() == m_subscribersCollection.size() + m_publishersCollection.size() && "the collections are not ok!");
		for (const auto& it : allSourcesInVO)
		{
			const auto& targetCollection = it.second.sourceType == ST_PUBLISHER ? m_publishersCollection : m_subscribersCollection;
			assert(targetCollection.find(it.first) != targetCollection.end() && "the node is not in the right collection");
		}
	}

	// Second check: ------ Check if links and refcounting are ok
	{
		// Iterate over all subscriber connection links and gather refcount for each used mirror node
		std::unordered_set<MirrorNodeInfo, MirrorNodeInfoHasher> mirrorNodesByLinks;
		std::map<TablePos, std::set<TablePos>> publisherPosToMirrorNodesByLinks;
		mirrorNodesByLinks.reserve(m_mirrorNodesCollection.size()); // We expect about this size..

		for (const TablePos& subscriberPos : m_subscribersCollection)
		{
			const SourceInfo& srcInfo = parent->m_posToSourceMap[subscriberPos];

			for (const auto& it : srcInfo.m_connectedTo)
			{
				const SourceInfo::LinkInfo& linkInfo = it.second;
				const TablePos& connectedPublisher = it.first;

				for (const TablePos& mirrorPos : linkInfo.mirrorNodesUsed)
				{
					MirrorNodeInfo mirrorInfo;
					mirrorInfo.nodePos = mirrorPos;
					mirrorInfo.parentPublisherPos = connectedPublisher;

					// THe first data structure under test
					auto it = mirrorNodesByLinks.find(mirrorInfo);
					if (it != mirrorNodesByLinks.end())
					{
						MirrorNodeInfo& existingMirrorInfo = const_cast<MirrorNodeInfo&>(*it);
						existingMirrorInfo.addSubscriber(subscriberPos);
					}
					else
					{
						mirrorInfo.addSubscriber(subscriberPos);
						mirrorNodesByLinks.insert(mirrorInfo);
					}

					// The second data structure under test
					{
						auto publishersToMirrorsIter = publisherPosToMirrorNodesByLinks.find(connectedPublisher);
						if (publishersToMirrorsIter == publisherPosToMirrorNodesByLinks.end())
						{
							std::set<TablePos> setOfMirrors = { mirrorPos };
							publisherPosToMirrorNodesByLinks.insert(std::make_pair(connectedPublisher, setOfMirrors));
							publishersToMirrorsIter = publisherPosToMirrorNodesByLinks.find(connectedPublisher);
						}

						if (publishersToMirrorsIter->second.find(mirrorPos) == publishersToMirrorsIter->second.end())
						{
							publishersToMirrorsIter->second.insert(mirrorPos);
						}
					}
				}
			}
		}

		// Check if both collections are the same
		{
			assert(mirrorNodesByLinks.size() == m_mirrorNodesCollection.size());
			for (const MirrorNodeInfo& mirrorInLinks : mirrorNodesByLinks)
			{
				const auto& it = m_mirrorNodesCollection.find(mirrorInLinks);
				assert(it != m_mirrorNodesCollection.end() && " this mirror node doesn't exist...");

				const MirrorNodeInfo& mirrorOnBoard = *it;
				assert(mirrorOnBoard == mirrorInLinks);
			}
		}
		// Check m_publisherPosToMirrorNodes and its copy
		{
			assert(m_publisherPosToMirrorNodes.size() != publisherPosToMirrorNodesByLinks.size());
			for (const auto&it : m_publisherPosToMirrorNodes)
			{
				const TablePos& publisherPos_board = it.first;
				const std::set<TablePos>& setOfMirrors_board = it.second;

				const auto& it_byLinks = publisherPosToMirrorNodesByLinks.find(publisherPos_board);
				assert(it_byLinks != publisherPosToMirrorNodesByLinks.end());

				const std::set<TablePos>& setOfMirrors_byLinks = it_byLinks->second;
				assert(setOfMirrors_board.size() == setOfMirrors_byLinks.size());

				for (const TablePos& posInBoard : setOfMirrors_board)
				{
					assert(setOfMirrors_byLinks.find(posInBoard) != setOfMirrors_byLinks.end());
				}
			}
		}
	}
}

void BoardObject::BoardPublisherSubscriberManager::printDetails(std::ostream& outStream)
{
	// Print publishers then consumers
	outStream << " === Publishers: " << std::endl;
	for (const TablePos& publisherPos : m_publishersCollection)
	{
		outStream << "##" << publisherPos << " Mirror nodes: ";
		const auto& it = m_publisherPosToMirrorNodes.find(publisherPos);
		if (it != m_publisherPosToMirrorNodes.end())
		{
			for (const auto& mirrorsIter : it->second)
			{
				outStream << mirrorsIter << " ";
			}
		}
		else
		{
			outStream << "None";
		}

		outStream << std::endl << "Subscribers serving: " << std::endl;
		
		for (const auto& it : parent->m_posToSourceMap[publisherPos].m_connectedTo)
		{
			const TablePos& subscriber = it.first;
			const SourceInfo::LinkInfo& linkInfo = it.second;
			
			outStream << subscriber << "flow: " << linkInfo.flow;
			if (linkInfo.mirrorNodesUsed.empty())
			{
				outStream << " Direct connection";
			}
			else
			{
				for (const TablePos& pathNode : linkInfo.mirrorNodesUsed)
				{
					outStream << pathNode << " ";
				}
			}

			outStream << std::endl;
		}

		outStream << std::endl;
	}

	outStream << " === Subscribers: " << std::endl;
	for (const TablePos& publisherPos : m_subscribersCollection)
	{
		outStream << publisherPos << " ";
	}

	outStream << std::endl;

}

// Collects all nodes' positions in the VO
void BoardObject::BoardPublisherSubscriberManager::collectAllNodePositionsInVO(std::vector<TablePos>& outNodes)
{
	std::vector<Cell*> outCells(MAX_ROWS * MAX_COLS);
	parent->collectAllNodesFromRoot(parent->getRootCell(), outCells);

	outNodes.clear();
	for (const Cell* cell : outCells)
	{
		TablePos cellPos(cell->m_row, cell->m_column);
		outNodes.push_back(cellPos);
	}
}

// Gets the closest VONode and distance to it
void BoardObject::BoardPublisherSubscriberManager::closestVONodeToPosition(const TablePos& queryPos, const std::vector<TablePos>& voNodesPos, TablePos& outClosestNode,  int& outClosestDistance)
{
	outClosestNode = TablePos(INVALID_POS, INVALID_POS);
	outClosestDistance = std::numeric_limits<int>::min();
	for (const TablePos& node : voNodesPos)
	{
		const int dist = manhattanDist(queryPos, node);
		if (dist < outClosestDistance)
		{
			outClosestDistance = dist;
			outClosestNode = node;
		}
	}
}

#endif
