/*
* author: wangxing
* date: 2017.12.12
*/

#ifndef MAX_FLOW_H
#define MAX_FLOW_H

#include "topology.h"

namespace ns3{

class MaxFlow
{
public:
	static TypeId GetTypeId (void);
	MaxFlow();
	~MaxFlow();
	
	void Calculate(Ptr<Topology> topo, uint16_t depth, uint16_t max);

	void ShowSolution(std::ostream &out = std::cout);
	void Solution(std::map<uint16_t, std::set<uint16_t> > &solution);

	void OutputFile(uint16_t max);

private:
	void Initialize(Ptr<Topology> topo, uint16_t depth, uint16_t max);
	
	void AddTag(void);
	int AdjustFlow(void);
	
	void ResetNodeFlag(void);
	void ReleaseFlow(uint16_t node);
	std::vector<uint16_t> GetChooseNode(void);
	bool IsCoverAll(void);

	void ShowNode(void);
	void ShowEdge(void);

	struct FlowEdge {
		uint16_t dst;
		uint16_t cost;
		uint16_t cap;
		uint16_t flow;
		FlowEdge *next;

		FlowEdge() { cost = 0; cap = 1; flow = 0; next = NULL; }
	};

	struct FlowNode {
		bool choose;
		uint16_t flag;
		uint16_t left;
		uint16_t cost;
		FlowEdge *first;

		FlowNode() {choose = true; flag = UINT16_MAX; left = 0; cost = UINT16_MAX; first = NULL; }
	};

	uint16_t m_src;
	uint16_t m_dst;
	uint16_t m_nodeNum;
	uint16_t m_edgeNum;
	uint16_t m_totalNum;

	Ptr<Topology> m_topo;
	std::vector<FlowNode> m_nodes;
};

}	// namespace ns3

#endif