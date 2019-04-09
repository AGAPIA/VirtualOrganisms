#include "BoardObject.h"
#include "Utils.h"
#include <unordered_map>
#include <assert.h>
#include "ExprGenerator.h"
#include <algorithm>
#include <stack>
#include <algorithm>
#include <set>

extern int g_maxDistanceToConnectNodes;

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
void BoardObject::BoardPublisherSubscriberManager::collectNodesForMirroring(std::unordered_set<TablePos, TablePosHasher>& outNodes)
{
	std::vector<Cell*> outCells(MAX_ROWS * MAX_COLS);
	parent->collectAllNodesFromRoot(parent->getRootCell(), outCells);

	outNodes.clear();
	for (const Cell* cell : outCells)
	{
		TablePos cellPos(cell->m_row, cell->m_column);

		// If the node is not used as mirror yet..
		if (m_mirrorNodesCollection.find(MirrorNodeInfo(cellPos)) == m_mirrorNodesCollection.end())
		{
			outNodes.insert(cellPos);
		}
	}
}

// Gets the closest VONode and distance to it
void BoardObject::BoardPublisherSubscriberManager::closestVONodeToPosition(const TablePos& queryPos, const std::unordered_set<TablePos, TablePosHasher>& voNodesPos, TablePos& outClosestNode,  int& outClosestDistance)
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

int compareByRemainingPower(std::pair<TablePos, float>& pos1, std::pair<TablePos, float>& pos2)
{
	const float power1 = pos1.second;
	const float power2 = pos2.second;

	if (power1 < power2)
		return -1;
	if (power1 > power2)
		return 1;

	return 0;
}

void BoardObject::BoardPublisherSubscriberManager::sortAndFilterPublishersAndSubscribers(std::vector<TablePos>& subscribers, std::vector<TablePos>& publishers, std::unordered_set<TablePos, TablePosHasher>& voNodes, const bool forMirroring)
{
	const std::unordered_set<TablePos, TablePosHasher>* inputCollection[2]	= { &m_publishersCollection, &m_subscribersCollection };
	std::vector<TablePos>* outputCollection[2]								= { &publishers, &subscribers };
	for (int i = 0; i < 2; i++)
	{
		const std::unordered_set<TablePos, TablePosHasher>& input	= *inputCollection[i];
		std::vector<TablePos>& output								= *outputCollection[i];
		output.clear();

		std::vector<std::pair<TablePos, float>>	output_extended(output.size());

		output_extended.clear();
		for (const auto& inputIt : input)
		{
			const SourceInfo& srcInfo = parent->m_posToSourceMap[inputIt];
			const float remainingPower = srcInfo.getRemainingPower();
			if (remainingPower > 0)
			{
				bool shouldAdd = false;
				if (forMirroring == false)
				{
					shouldAdd = true;
				}
				else
				{
					// If mirroring filtering is requested, check the distance to VO too
					TablePos outClosestVONode;
					int outClosestDistance = 0;
					closestVONodeToPosition(inputIt, voNodes, outClosestVONode, outClosestDistance);
					
					if (outClosestDistance <= g_maxDistanceToConnectNodes)
					{
						shouldAdd = true;
					}
				}

				if (shouldAdd)
				{
					output_extended.push_back(std::make_pair(inputIt, remainingPower));
				}
			}
		}

		std::sort(output_extended.begin(), output_extended.end(), ::compareByRemainingPower);

		for (const auto& it : output_extended)
			output.push_back(it.first);
	}
}

void BoardObject::BoardPublisherSubscriberManager::solveDirectConnections(std::vector<TablePos>& subscribers, std::vector<TablePos>& publishers)
{
	for (const TablePos& publisherPos : publishers)
	{
		SourceInfo& publisherSrcInfo = parent->m_posToSourceMap[publisherPos];
		assert(publisherSrcInfo.getRemainingPower() > 0);

		for (const TablePos& subscriberPos : subscribers)
		{
			SourceInfo& subscriberSrcInfo = parent->m_posToSourceMap[publisherPos];
			assert(subscriberSrcInfo.getRemainingPower() > 0);

			// Is the right service ?
			if (subscriberSrcInfo.serviceType != publisherSrcInfo.serviceType)
				continue;

			// Can the publisher satisfy this subscriber directly ?
			if (manhattanDist(publisherPos, subscriberPos) > g_maxDistanceToConnectNodes)
				continue;

			SourceInfo::LinkInfo linkInfo;
			linkInfo.flow = std::min(publisherSrcInfo.getRemainingPower(), subscriberSrcInfo.getRemainingPower());
			linkInfo.mirrorNodesUsed.clear(); // No mirror nodes used, direct connection

			publisherSrcInfo.AddConnectionFromPublisherToSubscriber(publisherPos, subscriberPos, subscriberSrcInfo, linkInfo);

			assert(publisherSrcInfo.getRemainingPower() >= 0);
			assert(subscriberSrcInfo.getRemainingPower() >= 0);
			
			// This publisher power is over, go to the next one
			if (publisherSrcInfo.getRemainingPower() == 0)
			{
				break;
			}
		}
	}
}

bool BoardObject::BoardPublisherSubscriberManager::connectNodesByHeuristic(const TablePos& startPos, const TablePos& subscriberPos, const std::unordered_set<TablePos, TablePosHasher>& mirroringSuitableNodes, std::vector<TablePos> outPath)
{
	// Just some sanity checks...
	assert((m_publishersCollection.find(startPos) != m_publishersCollection.end() || m_mirrorNodesCollection.find(MirrorNodeInfo(startPos)) != m_mirrorNodesCollection.end()) && "The start pos isn't a publisher or mirror ");
	assert(m_subscribersCollection.find(subscriberPos) != m_subscribersCollection.end() && "the end node is not a subscriber");
	
	// A bfs style search with heuristics
	TablePos currIterMirror = startPos;
	outPath.push_back(currIterMirror);

	std::unordered_set<TablePos, TablePosHasher> alreadyUsedNodes;
	alreadyUsedNodes.insert(startPos);

	while (true)
	{
		// If the distance to subscriber is good enough, exit
		if (manhattanDist(currIterMirror, subscriberPos) <= g_maxDistanceToConnectNodes)
		{
			return true;
		}

		// Get the closest mirror suitable node to subscrierPos as first key, as secondary key the one that is furthest from startPos (requirement: distance to it <= MAX_DIST
		TablePos bestNode(INVALID_POS, INVALID_POS);
		int bestNodeDistanceToSubscriber = std::numeric_limits<int>::max();
		int bestNodeDistanceToIterPos = std::numeric_limits<int>::min();
		for (const TablePos& node : mirroringSuitableNodes)
		{
			// Is node already used ?
			if (alreadyUsedNodes.find(node) != alreadyUsedNodes.end())
				continue;

			const int distToIterNode = manhattanDist(node, currIterMirror);
			if (distToIterNode > g_maxDistanceToConnectNodes)
				continue;

			const int distToSubscriberNode = manhattanDist(node, subscriberPos);
			if ((distToSubscriberNode < bestNodeDistanceToSubscriber) || (distToSubscriberNode == bestNodeDistanceToSubscriber && bestNodeDistanceToIterPos < distToIterNode))
			{
				bestNode = node;
				bestNodeDistanceToSubscriber = distToSubscriberNode;
				bestNodeDistanceToIterPos = distToIterNode;
			}
		}

		// If no node could be found...
		if (bestNode.row == INVALID_POS || bestNode.col == INVALID_POS)
			return false;

		// Continue movement from the best node found
		outPath.push_back(bestNode);
		alreadyUsedNodes.insert(bestNode);
		currIterMirror = bestNode;
	}
}

void BoardObject::BoardPublisherSubscriberManager::solveMirrorConnections(std::vector<TablePos>& subscribers, std::vector<TablePos>& publishers, std::unordered_set<TablePos, TablePosHasher>& mirroringSuitableNodes)
{
	for (const TablePos& publisherPos : publishers)
	{
		SourceInfo& publisherSrcInfo = parent->m_posToSourceMap[publisherPos];
		assert(publisherSrcInfo.getRemainingPower() > 0);

		for (const TablePos& subscriberPos : subscribers)
		{
			SourceInfo& subscriberSrcInfo = parent->m_posToSourceMap[publisherPos];
			assert(subscriberSrcInfo.getRemainingPower() > 0);

			// Is the right service ?
			if (subscriberSrcInfo.serviceType != publisherSrcInfo.serviceType)
				continue;

			// Connect them by mirroring below. 
			const int publisherToSubscriberDist = manhattanDist(publisherPos, subscriberPos);
			assert(publisherToSubscriberDist > g_maxDistanceToConnectNodes && " This pair failed to connect directly at the call before !");

			// Get the closest start point node for connecting the publisher to subscriber (either the publisher itself or one of its mirrors
			TablePos startPos				= publisherPos;
			int startPosDistToSubscriber	= publisherToSubscriberDist;
			for (const TablePos& mirrorPos : m_publisherPosToMirrorNodes[publisherPos])
			{
				const int distMirrorToSubscriber = manhattanDist(mirrorPos, subscriberPos);
				if (distMirrorToSubscriber < startPosDistToSubscriber)
				{
					startPos					= mirrorPos;
					startPosDistToSubscriber	= distMirrorToSubscriber;
				}
			}

			// Connect the two nodes
			std::vector<TablePos> pathWithMirrors;
			bool res = connectNodesByHeuristic(startPos, subscriberPos, mirroringSuitableNodes, pathWithMirrors);
			if (res)
			{
				// If connection with mirrors succeeded:

				// Create the connection between them
				SourceInfo::LinkInfo linkInfo;
				linkInfo.flow = std::min(publisherSrcInfo.getRemainingPower(), subscriberSrcInfo.getRemainingPower());
				linkInfo.mirrorNodesUsed.clear();
				for (const TablePos& nodePos : pathWithMirrors)
				{
					if (nodePos != publisherPos)
						linkInfo.mirrorNodesUsed.push_back(nodePos);
				}
				publisherSrcInfo.AddConnectionFromPublisherToSubscriber(publisherPos, subscriberPos, subscriberSrcInfo, linkInfo);


				// Remove the nodes used from suitable for mirroring
				// And add them to the mirror data structures
				for (const TablePos& mirrorUsedPos : pathWithMirrors)
				{
					const auto& it = mirroringSuitableNodes.find(mirrorUsedPos);
					if (it != mirroringSuitableNodes.end())
					{
						mirroringSuitableNodes.erase(it);

						// If not already marked in the publisher mirror list, add it there
						auto& mirrorNodesForPublisher = m_publisherPosToMirrorNodes[publisherPos];
						if (mirrorNodesForPublisher.find(mirrorUsedPos) == mirrorNodesForPublisher.end())
							mirrorNodesForPublisher.insert(mirrorUsedPos);

						// Add in the other data structures collecting all mirror nodes details
						auto& it = m_mirrorNodesCollection.find(MirrorNodeInfo(mirrorUsedPos));
						if (it == m_mirrorNodesCollection.end())
						{
							// New entry, first time mirror node used
							MirrorNodeInfo mirrorInfo;
							mirrorInfo.nodePos = mirrorUsedPos;
							mirrorInfo.parentPublisherPos = publisherPos;
							m_mirrorNodesCollection.insert(mirrorInfo);

							it = m_mirrorNodesCollection.find(MirrorNodeInfo(mirrorUsedPos));
						}
						MirrorNodeInfo& mirrorNodeInfo = const_cast<MirrorNodeInfo&>(*it);
						mirrorNodeInfo.addSubscriber(subscriberPos);
					}
					else
					{
						// As a sanity check, only if publisher was starting point he couldn't ve in the mirrorsSuitableNodes
						assert(*it == publisherPos && "A mirror node was used but can't be found in the data structures...");
					}
				}
			}

			assert(publisherSrcInfo.getRemainingPower() >= 0);
			assert(subscriberSrcInfo.getRemainingPower() >= 0);

			// This publisher power is over, go to the next one
			if (publisherSrcInfo.getRemainingPower() == 0)
			{
				break;
			}
		}
	}
}

void BoardObject::BoardPublisherSubscriberManager::solvePublishersSubscribersConnections(std::ostream* outStream)
{
	// Collect the nodes that are suitable for mirroring
	std::unordered_set<TablePos, TablePosHasher> mirroringSuitableNodes(MAX_COLS*MAX_ROWS);
	collectNodesForMirroring(mirroringSuitableNodes);

	// Do sorting first - sort subscribers and publishers by their remaining capacity, and filter those which have cap 0
	std::vector<TablePos> subscribers(m_subscribersCollection.size());
	std::vector<TablePos> publishers(m_publishersCollection.size());
	sortAndFilterPublishersAndSubscribers(subscribers, publishers, mirroringSuitableNodes, false);
	solveDirectConnections(subscribers, publishers);

	sortAndFilterPublishersAndSubscribers(subscribers, publishers, mirroringSuitableNodes, true);
	solveMirrorConnections(subscribers, publishers, mirroringSuitableNodes);

	do_sanityChecks();
}

float BoardObject::BoardPublisherSubscriberManager::getCurrentPowerUsed() const
{
	float totalUsedPower = 0.0f;
	for (const auto& it : parent->m_posToSourceMap)
	{
		const SourceInfo& srcInfo = it.second;
		if (srcInfo.sourceType != ST_PUBLISHER)
			continue;

		totalUsedPower += srcInfo.getUsedPower();
	}

	return totalUsedPower;
}


#endif
