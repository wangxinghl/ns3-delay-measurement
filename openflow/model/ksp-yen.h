/**
 * @author: wangxing
 * @date: 2018.01.29
 */

#ifndef KSP_YEN_H
#define KSP_YEN_H

#include "topology.h"

namespace ns3 {

class KSPYen
{
public:
	static TypeId GetTypeId(void);

	KSPYen(Ptr<Topology> topo, uint16_t k = 3);
	~KSPYen();
	
	void LoadKPaths(Paths_t **allPaths);

private:
	Path_t GetShortestPath(uint16_t src, uint16_t dst, Paths_t &cur);

	Ptr<Topology> m_topo;
	uint16_t m_k;
	std::vector<std::vector<uint8_t> > m_adjacent;
};

}	// namespace ns3

#endif  /* TOPOLOGY_H */