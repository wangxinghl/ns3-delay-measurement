/*
* author: wangxing
* date: 2017.12.12
*/

#include "max-flow.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MaxFlow");

NS_OBJECT_ENSURE_REGISTERED(MaxFlow);

TypeId MaxFlow::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MaxFlow")
    .SetParent<Object> ()
    .SetGroupName ("Openflow")
  ;
  return tid;
}

MaxFlow::MaxFlow()
{
	m_topo = NULL;
	m_nodeNum = 0;
	m_edgeNum = 0;
	m_totalNum = 0;
	m_src = UINT16_MAX;
	m_dst = UINT16_MAX;
}

MaxFlow::~MaxFlow()
{
	m_adj.clear();
	m_nodes.clear();
	m_edges.clear();
}

void MaxFlow::Calculate(Ptr<Topology> topo, uint16_t depth, uint16_t max)
{
	Initialize(topo, depth, max);	// initialize the flow topology
	MaxFlowCalculate();		// get the initial results

	std::vector<FlowEdge> edges_copy;
	while (1) {
		edges_copy = m_edges;
		std::vector<uint16_t> choosed = GetChoosedNode();
		while (!choosed.empty()) {
			uint16_t first = choosed[0];
			ReleaseFlow(first);
			m_edges[m_nodes[first].adj].cap = 0;
			MaxFlowCalculate();

			if (IsCoverAll()) break;
			
			choosed.erase(choosed.begin());
			m_edges = edges_copy;
		}
		if (choosed.empty()) break;
	}
	NS_LOG_INFO("Calculate over: node " << m_nodeNum << " edge" << m_edgeNum);
}

std::map<uint16_t, std::set<uint16_t> > MaxFlow::Solution(bool show)
{
	std::map<uint16_t, std::set<uint16_t> > solution;		// map<node_id, set<link_id> >

	uint16_t adj;
	for (uint16_t i = 0; i < m_edgeNum; i++) {	// Get solution
		adj = m_nodes[i].adj;
		while (adj != UINT16_MAX && m_edges[adj].flow == 0)
			adj = m_edges[adj].next;
		NS_ASSERT(adj != UINT16_MAX);	// p can't be NULL
		solution[m_edges[adj].dst - m_edgeNum].insert(i);
	}

	if (show) {
		std::cout << "result = " << solution.size() << std::endl;
		for (std::map<uint16_t, std::set<uint16_t> >::iterator it = solution.begin(); it != solution.end(); it++) {
			std::cout << "switch " << it->first << "(" << it->second.size() << "): ";
			for (std::set<uint16_t>::iterator iter = it->second.begin(); iter != it->second.end(); iter++) {
				Edge &edge = m_topo->m_edges[*iter];
				std::cout << "<" << edge.src << "," << edge.dst << ">, ";
			}
			std::cout << std::endl;
		}
	}
	return solution;
}

void MaxFlow::Initialize(Ptr<Topology> topo, uint16_t depth, uint16_t max)
{
	m_topo = topo;
	m_nodeNum = m_topo->m_numSw;
	m_edgeNum = m_topo->GetSwitchEdgeNum();
	m_totalNum = m_nodeNum + m_edgeNum + 2;	// Total flow node = edge + node + 2 (source node and destination node)
	m_src = m_totalNum - 2;
	m_dst = m_totalNum - 1;

	std::vector<std::map<uint16_t, uint16_t> > edge2node;		// Edge to node
	for (uint16_t i = 0; i < m_edgeNum; i++)
		edge2node.push_back(m_topo->GetEdgeAdjacentNode(i, depth));

	/*
	* Create Flow Topology
	*/
	m_nodes.resize(m_totalNum);		// Initialize the nodes

	m_nodes[m_src].adj = 0;		// From source node to the first layer
	for (uint16_t i = 0; i < m_edgeNum; i++) {
		m_adj[m_src][i] = m_edges.size();
		FlowEdge edge;
		edge.dst = i;
		edge.next = m_edges.size() + 1;
		m_edges.push_back(edge);
	}
	m_edges.back().next = UINT16_MAX;

	for (uint16_t i = 0; i < m_edgeNum; i++) {	// From the first layer to the second layer
		m_nodes[i].adj = m_edges.size();
		std::map<uint16_t, uint16_t> &temp = edge2node[i];
		for (std::map<uint16_t, uint16_t>::iterator it = temp.begin(); it != temp.end(); it++) {
			m_adj[i][m_edgeNum + it->first] = m_edges.size();
			FlowEdge edge;
			edge.dst = m_edgeNum + it->first;
			edge.cost = it->second;
			edge.next = m_edges.size() + 1;
			m_edges.push_back(edge);
		}
		m_edges.back().next = UINT16_MAX;
	}

	for (uint16_t i = m_edgeNum; i < m_src; i++) {	// From the second layer to destination node
		m_nodes[i].adj = m_edges.size();
		m_adj[i][m_dst] = m_edges.size();
		FlowEdge edge;
		edge.dst = m_dst;
		//edge.cost = UINT16_MAX;	// need to rewrite
		edge.cap = max;
		m_edges.push_back(edge);
	}
}

void MaxFlow::MaxFlowCalculate(void)
{
	while (1) {
		ResetFlag();
		AddTag();
		if (m_nodes[m_dst].flag == UINT16_MAX) break;
		Adjust();
	}
}

void MaxFlow::AddTag(void)
{
	uint16_t adj;
	FlowEdge *p, *q;
	
	/* from source node to the first layer */
	adj = m_nodes[m_src].adj;
	while (adj != UINT16_MAX) {
		p = &m_edges[adj];
		if (p->flow < p->cap) {
			m_nodes[p->dst].flag = m_src;
			m_nodes[p->dst].left = 1;
		}
		adj = p->next;
	}
	
	/* from the first layer to the second layer */
	for (uint16_t i = 0; i < m_edgeNum; i++) {
		if (m_nodes[i].flag != UINT16_MAX) {	// link have flag
			adj = m_nodes[i].adj;
			while (adj != UINT16_MAX) {
				p = &m_edges[adj];		// the first layer ---> the second layer
				q = &m_edges[m_nodes[p->dst].adj];	// the second layer ---> destination
				if (p->flow < p->cap && q->flow < q->cap) {	// ÓÐ¿ÉÓÃÈÝÁ¿£¬Ä¿µÄ½Úµã¿ÉÐÐ±¸Ñ¡½Úµã
					if (m_nodes[p->dst].flag == UINT16_MAX) {  // have no flag, give flag
						m_nodes[p->dst].flag = i;
						m_nodes[p->dst].left = 1;
						m_nodes[p->dst].cost = p->cost;
					}
					else if (p->cost < m_nodes[p->dst].cost) {	// have flag, modify flag. minimize distance
						m_nodes[p->dst].flag = i;
						m_nodes[p->dst].cost = p->cost;
					}
				}
				adj = p->next;
			}
		}
	}

	/* from the second layer to destination node */
	for (uint16_t i = m_edgeNum; i < m_src; i++) {
		if (m_nodes[i].flag != UINT16_MAX ) {
			if (m_nodes[m_dst].flag == UINT16_MAX) {
				m_nodes[m_dst].flag = i;
				m_nodes[m_dst].left = 1;
				m_nodes[m_dst].cost = m_nodes[i].cost;
			}
			else if (m_nodes[i].cost < m_nodes[m_dst].cost) {
				m_nodes[m_dst].flag = i;
				m_nodes[m_dst].cost = m_nodes[i].cost;
			}
		}
	}
}

void MaxFlow::Adjust(void)
{
	uint16_t cur = m_dst;
	while (cur != m_src) {
		m_edges[m_adj[m_nodes[cur].flag][cur]].flow++;
		cur = m_nodes[cur].flag;
	}
}

void MaxFlow::ResetFlag(void)
{
	for (uint16_t i = 0; i < m_totalNum; i++)
		m_nodes[i].flag = UINT16_MAX;
}

void MaxFlow::ReleaseFlow(uint16_t node)
{
	/* from the second layer to destination node */
	m_edges[m_nodes[node].adj].flow = 0;

	std::set<uint16_t> temp;
	/* from the first layer to the second layer */
	uint16_t adj;
	FlowEdge *p;
	for (uint16_t i = 0; i < m_edgeNum; i++) {
		adj = m_nodes[i].adj;
		while (adj != UINT16_MAX) {
			p = &m_edges[adj];
			if (p->dst == node && p->flow == 1) {
				p->flow = 0;
				temp.insert(i);
				break;
			}
			adj = p->next;
		}
	}
	
	/* from source node to the first layer */
	adj = m_nodes[m_src].adj;
	while (adj != UINT16_MAX) {
		p = &m_edges[adj];
		if (temp.find(p->dst) != temp.end()) {
			p->flow = 0;
		}
		adj = p->next;
	}
}

std::vector<uint16_t> MaxFlow::GetChoosedNode(void)
{
	std::vector<uint16_t> choose;
	for (uint16_t i = m_edgeNum; i < m_src; i++) {
		if (m_edges[m_nodes[i].adj].flow != 0)
			choose.push_back(i);
	}
	
	// sort
	uint16_t size = choose.size();
	for (uint16_t i = 0; i < size; ++i) {
		for (uint16_t j = i + 1; j < size; ++j) {
			if (m_edges[m_nodes[choose[i]].adj].flow > m_edges[m_nodes[choose[j]].adj].flow) {
				uint16_t temp = choose[i];
				choose[i] = choose[j];
				choose[j] = temp;
			}
		}
	}
	return choose;
}

bool MaxFlow::IsCoverAll(void)
{
	uint16_t adj = m_nodes[m_src].adj;
	while (adj != UINT16_MAX){
		if (m_edges[adj].flow == 0)
			return false;
		adj = m_edges[adj].next;
	}
	return true;
}

}	// namespace ns3