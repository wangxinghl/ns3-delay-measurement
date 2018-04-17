/*
* author: wangxing
* date: 2017.12.12
*/

#ifndef MAX_FLOW_H
#define MAX_FLOW_H

#include "topology.h"

namespace ns3 {

class MaxFlow
{
public:
	static TypeId GetTypeId (void);
	MaxFlow();
	~MaxFlow();
	
	void Calculate(Ptr<Topology> topo, uint16_t depth, uint16_t max);
	std::map<uint16_t, std::set<uint16_t> > Solution(bool show = false);

private:
	void Initialize(Ptr<Topology> topo, uint16_t depth, uint16_t max);
	
	void MaxFlowCalculate(void);
	void AddTag(void);
	void Adjust(void);
	void ResetFlag(void);
	void ReleaseFlow(uint16_t node);
	std::vector<uint16_t> GetChoosedNode(void);
	bool IsCoverAll(void);

	struct FlowEdge {
		uint16_t dst;
		uint16_t cost;
		uint8_t cap;
		uint8_t flow;
		uint16_t next;

		FlowEdge() { cost = 0; cap = 1; flow = 0; next = UINT16_MAX; }
	};

	struct FlowNode {
		uint16_t flag;
		uint16_t left;
		uint16_t cost;
		uint16_t adj;

		FlowNode() { flag = UINT16_MAX; left = 0; cost = UINT16_MAX; adj = UINT16_MAX; }
	};

	Ptr<Topology> m_topo;
	uint16_t m_nodeNum;
	uint16_t m_edgeNum;
	uint16_t m_totalNum;
	uint16_t m_src;
	uint16_t m_dst;
	
	std::map<uint16_t, std::map<uint16_t, uint16_t> > m_adj;
	std::vector<FlowNode> m_nodes;
	std::vector<FlowEdge> m_edges;
};

}	// namespace ns3

#endif