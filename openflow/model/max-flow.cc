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
	m_nodeNum = 0;
	m_edgeNum = 0;
	m_totalNum = 0;
	m_src = UINT16_MAX;
	m_dst = UINT16_MAX;
	m_topo = NULL;
}

MaxFlow::~MaxFlow()
{
	FlowEdge *cur, *next;
	for (uint16_t i = 0; i < m_totalNum; i++) {
		cur = m_nodes[i].first;
		while (cur) {
			next = cur->next;
			delete cur;
			cur = next;
		}
	}
}

void MaxFlow::Calculate(Ptr<Topology> topo, uint16_t depth, uint16_t max)
{
	// initialize the flow topology
	Initialize(topo, depth, max);

	// put flow
	while (1) {
		ResetNodeFlag();
		AddTag();
		if (AdjustFlow() == 1) break;
	}

	// try to push flow
	while (1) {
		std::vector<uint16_t> choose = GetChooseNode();
		std::vector<uint16_t>::iterator it = choose.begin();
		for (; it != choose.end(); it++) {
			m_nodes[*it].choose = false;
			ReleaseFlow(*it);
			while (1) {
				ResetNodeFlag();
				AddTag();
				if (AdjustFlow() == 1) break;
			}
			if (IsCoverAll()) break;
			
			m_nodes[*it].choose = true;
			while (1) {
				ResetNodeFlag();
				AddTag();
				if (AdjustFlow() == 1) break;
			}
		}
		if (it == choose.end()) break;
	}
	NS_LOG_INFO("Calculate over: node " << m_nodeNum << " edge" << m_edgeNum);
}

void MaxFlow::ShowSolution(std::ostream &out)
{
	std::map<uint16_t, std::vector<uint16_t> > result;
	for (uint16_t i = 0; i < m_edgeNum; i++) {	// Get result
		FlowEdge *p = m_nodes[i].first;
		while (p && p->flow == 0)
			p = p->next;

		NS_ASSERT(p != 0);
		result[p->dst - m_edgeNum + m_topo->m_numHost].push_back(i);
	}

	out << "result = " << result.size() << std::endl;
	std::map<uint16_t, std::vector<uint16_t> >::iterator it;
	for (it = result.begin(); it != result.end(); it++) {
		out << "node " << it->first << "(" << it->second.size() << "): ";
		for (std::vector<uint16_t>::iterator iter = it->second.begin(); iter != it->second.end(); iter++) {
			Edge &edge = m_topo->m_edges[*iter];
			out << *iter << "<" << edge.src << "," << edge.dst << ">, ";
			}
		out << std::endl;
	}
}

void MaxFlow::Solution(std::map<uint16_t, std::set<uint16_t> > &solution)
{
	for (uint16_t i = 0; i < m_edgeNum; i++) {	// Get solution
		FlowEdge *p = m_nodes[i].first;
		while (p && p->flow == 0)
			p = p->next;

		NS_ASSERT(p != 0);	// p can't be NULL
		solution[p->dst - m_edgeNum].insert(i);
	}
}

void MaxFlow::OutputFile(uint16_t max)
{
	std::ofstream fout("../cplex/maxflow/maxflow.dat");
	fout << "nodeN=" << m_nodeNum << ";\n";
	fout << "edgeM=" << m_edgeNum << ";\n";
	fout << "K=" << max << ";\n";
	fout << "delta=[\n";
	for (uint16_t i = 0; i < m_edgeNum; i++) {
		fout << "[";
		FlowEdge *p = m_nodes[i].first;
		for (uint16_t j = 0; j < m_nodeNum; j++) {
			if (p && p->dst - m_edgeNum == j) {
				fout << 1 << ",";
				p = p->next;
			}
			else {
				fout << 0 << ",";
			}
		}
		fout << "],\n";
	}
	fout << "];\n";
	fout.close();
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

	FlowEdge **cur, *next;
	cur = &m_nodes[m_src].first;		// From source node to the first layer
	for (uint16_t i = 0; i < m_edgeNum; i++) {
		next = new FlowEdge;
		next->dst = i;
		
		*cur = next;
		cur = &next->next;
	}

	for (uint16_t i = 0; i < m_edgeNum; i++) {	// From the first layer to the second layer
		cur = &m_nodes[i].first;
		std::map<uint16_t, uint16_t> &temp = edge2node[i];
		for (std::map<uint16_t, uint16_t>::iterator it = temp.begin(); it != temp.end(); it++) {
			next = new FlowEdge;
			next->dst = m_edgeNum + it->first;
			next->cost = it->second;

			*cur = next;
			cur = &next->next;
		}
	}

	for (uint16_t i = m_edgeNum; i < m_src; i++) {	// From the second layer to destination node
		next = new FlowEdge;
		next->dst = m_dst;
		// next->cost = IN_MAX;		// need to rewrite
		next->cap = max;
		m_nodes[i].first = next;
	}
}

void MaxFlow::AddTag(void)
{
	m_nodes[m_src].flag = m_src;
	m_nodes[m_src].left = UINT16_MAX;

	std::set<uint16_t> cur, next;
	cur.insert(m_src);
	while (!cur.empty()) {
		for (std::set<uint16_t>::iterator it = cur.begin(); it != cur.end(); it++) {
			std::map<uint16_t, std::vector<uint16_t> > temp;
			FlowEdge *p = m_nodes[*it].first;
			while (p) {		
				if (p->flow < p->cap && m_nodes[p->dst].choose)
					temp[p->cost].push_back(p->dst);
				p = p->next; 
			}
			if (temp.empty()) continue;

			uint16_t min_cost = UINT16_MAX;
			for (std::map<uint16_t, std::vector<uint16_t> >::iterator iter = temp.begin(); iter != temp.end(); iter++) {
				if (iter->first < min_cost)
					min_cost = iter->first;
			}

			for (std::vector<uint16_t>::iterator iter = temp[min_cost].begin(); iter != temp[min_cost].end(); iter++) {
				if (m_nodes[*iter].flag == UINT16_MAX) {
					m_nodes[*iter].flag = *it;
					m_nodes[*iter].left = 1;
					m_nodes[*iter].cost = min_cost;
					next.insert(*iter);
				}
				else {
					if (min_cost < m_nodes[*iter].cost) {
						m_nodes[*iter].flag = *it;
						m_nodes[*iter].cost = min_cost;
					}
				}
			}
		}
		cur = next;
		next.clear();
	}
}

int MaxFlow::AdjustFlow(void)
{
	if (m_nodes[m_dst].flag == UINT16_MAX)
		return 1;

	uint16_t cur = m_dst;
	uint16_t pre = m_nodes[cur].flag;
	while (cur != m_src) {
		FlowEdge *p = m_nodes[pre].first;
		while (p && p->dst != cur)	
			p = p->next;
		p->flow += m_nodes[cur].left;

		cur = pre;
		pre = m_nodes[pre].flag;
	}
	return 0;
}

void MaxFlow::ResetNodeFlag(void)
{
	for (uint16_t i = 0; i < m_totalNum; i++)
		m_nodes[i].flag = UINT16_MAX;
}

void MaxFlow::ReleaseFlow(uint16_t node)
{
	m_nodes[node].first->flow = 0;

	std::set<uint16_t> temp;
	for (uint16_t i = 0; i < m_edgeNum; i++) {
		FlowEdge *p = m_nodes[i].first;
		while (p) {
			if (p->dst == node && p->flow == 1) {
				p->flow = 0;
				temp.insert(i);
				break;
			}
			p = p->next;
		}
	}
	
	FlowEdge *p = m_nodes[m_src].first;
	while (p) {
		if (temp.find(p->dst) != temp.end())
			p->flow = 0;
		p = p->next;
	}
}

std::vector<uint16_t> MaxFlow::GetChooseNode(void)
{
	std::vector<uint16_t> choose;
	for (uint16_t i = m_edgeNum; i < m_src; i++) {
		if (m_nodes[i].first->flow != 0)
			choose.push_back(i);
	}
	
	// sort
	uint16_t size = choose.size();
	for (uint16_t i = 0; i < size; ++i) {
		for (uint16_t j = i + 1; j < size; ++j) {
			if (m_nodes[choose[i]].first->flow > m_nodes[choose[j]].first->flow) {
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
	FlowEdge *p = m_nodes[m_src].first;
	while (p && p->flow == 1)
		p = p->next;

	if (p)
		return false;
	else
		return true;
}

void MaxFlow::ShowNode(void)
{
	for (size_t i = 0; i < m_nodes.size(); i++)
		std::cout << i << " (" << m_nodes[i].flag << "," << m_nodes[i].left << ")\n";
}

void MaxFlow::ShowEdge(void)
{
	for (size_t i = 0; i < m_nodes.size(); i++) {
		FlowEdge *p = m_nodes[i].first;
		while (p) {
			if (p->flow != 0)
				std::cout << i << "-->" << p->dst << "(" << p->flow << ")\n";
			p = p->next;
		}
	}
}

}	// namespace ns3