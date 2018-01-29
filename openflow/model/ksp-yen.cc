/**
 * @author: wangxing
 * @date: 2018.01.29
 */

#include "ksp-yen.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("KSPYen");

NS_OBJECT_ENSURE_REGISTERED(KSPYen);

TypeId KSPYen::GetTypeId(void)
{
	static TypeId tid = TypeId ("ns3::KSPYen")
    .SetParent<Object> ()
    .SetGroupName ("Openflow")
  ;
  return tid;

}

KSPYen::KSPYen(Ptr<Topology> topo, uint16_t k)
{
	NS_LOG_FUNCTION (this);

	m_topo = topo;
	m_k = k;

	std::vector<uint8_t> temp(topo->m_numSw, 0);
	for (uint16_t i = 0; i < topo->m_numSw; ++i)
		m_adjacent.push_back(temp);

	for (uint16_t i = 0; i < topo->m_edges.size(); ++i) {
		if (topo->m_edges[i].ssFlag) {
			uint16_t src = topo->m_edges[i].src - topo->m_numHost;
			uint16_t dst = topo->m_edges[i].dst - topo->m_numHost;
			m_adjacent[src][dst] = 1;
		}
	}
}

KSPYen::~KSPYen()
{
	NS_LOG_FUNCTION (this);
	m_adjacent.clear();
}
	
void KSPYen::LoadKPaths(Paths_t **allPaths)
{
	NS_LOG_FUNCTION (this);

	for (uint16_t i = 0; i < m_topo->m_numHost; ++i) {
		for (uint16_t j = 0; j < m_topo->m_numHost; ++j) {
			if (i == j) continue;

			// get the first shortest path
			Path_t path = m_topo->Dijkstra(i, j);
			Paths_t &A =  allPaths[i][j];
			A.push_back(path);
			if (A.size() == 2) continue;

			// get the next k-1 path
			while(A.size() < m_k) {
				// get all the alternative path to B
				Paths_t B;
				Path_t pk = A.back();
				for (uint16_t k = 1; k < pk.size(); ++k) {	// k = 0: the switch connect to host directly
					Path_t pi = GetShortestPath(m_topo->m_edges[pk[k]].src, j, A);
					// Path_t pi = m_topo->Dijkstra(m_topo->m_edges[pk[k]].src, j);
					pi.insert(pi.end(), pk.begin() + k + 1, pk.end());
					B.push_back(pi);
				}

				// choose the shortest path in B to A
				uint16_t index;
				uint16_t min = -1;
				for (uint16_t k = 0; k < B.size(); ++k) {
					if (B[k].size() < min) {
						min = B[k].size();
						index = k;
					}
				}
				A.push_back(B[index]);
			}
		}
	}
}

Path_t KSPYen::GetShortestPath(uint16_t src, uint16_t dst, Paths_t &cur)
{
	NS_LOG_FUNCTION (this);
	Path_t path;


	return path;
}
	
}	// namespace ns3