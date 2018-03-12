/**
  * @author: wangxing
  * @date: 2018.01.15
  */

#include "ns3/log.h"
#include "max-flow.h"
#include "YenTopKShortestPathsAlg.h"
#include "simple-controller.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SimpleController");

NS_OBJECT_ENSURE_REGISTERED(SimpleController);

TypeId SimpleController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SimpleController")
    .SetParent<Object> ()
    .SetGroupName ("Openflow")
  ;
  return tid;
}

SimpleController::SimpleController ()
{
	NS_LOG_FUNCTION(this);

  m_rtt_file.open("scratch/output-file-rtt.txt");
  m_utilization_file.open("scratch/output-file-utilization.txt");
  m_tcam_file.open("scratch/output-file-tcam.txt");
  Simulator::Schedule (OUTPUT_FILE_PERIOD, &SimpleController::OutputFile, this);
}

SimpleController::~SimpleController ()
{
	NS_LOG_FUNCTION(this);

  m_swtches.clear();

  m_utilization_file.close();
  m_utilization.clear();
  
  m_rtt_file.close();
  m_rtt.clear();

  for (uint16_t i = 0; i < m_tcamNum.size(); ++i) {
    m_tcam_file << m_tcamNum[i] << " ";
  }
  m_tcam_file << std::endl;
  m_tcam_file.close();
  m_tcamNum.clear();
  
  m_curPath.clear();
  m_swPath.clear();
}

void SimpleController::SetTopology(Ptr<Topology> topo)
{
	NS_LOG_FUNCTION(this);
	m_topo = topo;
  m_edgeNum = m_topo->m_edges.size() / 2;
  m_switchEdgeNum = topo->GetSwitchEdgeNum();
  m_bandWidth = topo->GetBandwidth();

  // RTT
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    m_rtt.push_back(0);
    m_rtt_file << i << " ";
  }
  m_rtt_file << std::endl;
  // utilization
  m_utilization.resize(topo->m_edges.size() / 2);
  for (uint16_t i = 0; i < m_utilization.size(); ++i) {
    Edge &edge = topo->m_edges[i];
    m_utilization_file << "<" << edge.src << "," << edge.dst << "> ";
  }
  m_utilization_file << std::endl;
  // TCAM num = host_num + 5
  for (uint16_t i = 0; i < topo->m_numSw; ++i) {
    m_tcamNum.push_back(topo->m_numHost + 5);
    m_tcam_file << i << " ";
  }
  m_tcam_file << std::endl;

  /**
   * max-flow calculate
   */
  MaxFlow maxflow;
  maxflow.Calculate(topo, 2, 8);    // depth = 2, max = 8;
  std::map<uint16_t, std::set<uint16_t> > solution;
  maxflow.Solution(solution);
  // show result
  /*for (std::map<uint16_t, std::set<uint16_t> >::iterator iter = solution.begin(); iter != solution.end(); ++iter) {
    std::cout << "switch " << iter->first << "(" << iter->second.size() << "): ";
    for (std::set<uint16_t>::iterator it = iter->second.begin(); it != iter->second.end(); ++it) {
      Edge &edge = topo->m_edges[*it];
      std::cout << "<" << edge.src << "," << edge.dst << ">, ";
    }
    std::cout << std::endl;
  }*/
  StartDelayMeasure(solution);

  /**
   * Set data flow entry
   */
  // SetFlowEntry();
  SetSwitchToHostFlowEntry();
  for (uint16_t i = 0; i < m_tcamNum.size(); ++i) {
    m_tcam_file << m_tcamNum[i] << " ";
  }
  m_tcam_file << std::endl;

  /**
   * load banlance
   */
  StartLoadBanlance();
}

void SimpleController::AddSwitch (Ptr<OpenFlowSwitchNetDevice> swtch)
{
	NS_LOG_FUNCTION(this);
  if (m_switches.find (swtch) != m_switches.end ()) {
  	NS_LOG_INFO ("This Controller has already registered this switch!");
  	return;
  }
  m_switches.insert (swtch);
  m_swtches.push_back(swtch);
}

void SimpleController::ReceiveFromSwitch (Ptr<OpenFlowSwitchNetDevice> swtch, ofpbuf* buffer)
{
	if (m_switches.find(swtch) == m_switches.end()) {
    NS_LOG_ERROR ("Can't receive from this switch, not registered to the Controller.");
    return;
  }

  uint8_t type = GetPacketType(buffer);
  switch(type) {
    case OFPT_HELLO:
      ReceiveHello(buffer);
      break;
  	case OFPT_PACKET_IN:
  		ReceivePacketIn(buffer);
    	break;
  	case OFPT_PORT_STATUS:
  		ReceivePortStatus(buffer);
    	break;
  	default:
    	NS_LOG_ERROR ("Can't receive this ofp message!" << (int)type);
  }
}

void SimpleController::SetFlowEntry(void)
{
  NS_LOG_FUNCTION(this);
  
  for (uint16_t src = 0; src < m_topo->m_numHost; ++src) {
    for (uint16_t dst = 0; dst < m_topo->m_numHost; ++dst) {
      if (src == dst)
        continue;

      Path_t path = m_topo->Dijkstra(src, dst);   // get shortest path from src to dst
      for (uint16_t i = 1; i < path.size (); ++i) {
        uint16_t sw = m_topo->m_edges[path[i]].src - m_topo->m_numHost;   // get switch
        uint16_t out_port = m_topo->m_edges[path[i]].spt;   // get out port
        SendFlowEntry(sw, 0, src, dst, out_port);  // wildcards = 0
      }
    }
  }
}

void SimpleController::SetSwitchToHostFlowEntry(void)
{
  NS_LOG_FUNCTION(this);

  for (uint16_t sw = 0; sw < m_topo->m_numSw; ++sw) {
    std::vector<Path_t> paths;
    for (uint16_t host = 0; host < m_topo->m_numHost; ++host) {
      // get shortest path from switch to host
      Path_t path = m_topo->Dijkstra(sw + m_topo->m_numHost, host);
      paths.push_back(path);

      uint32_t wildcards = OFPFW_DL_SRC | OFPFW_NW_SRC_ALL;   // wildcards = OFPFW_DL_SRC | OFPFW_NW_SRC_ALL
      uint16_t out_port =  m_topo->m_edges[path[0]].spt;    // get out port
      SendFlowEntry(sw, wildcards, sw + m_topo->m_numHost, host, out_port);
    }
    m_swPath.push_back(paths);
  }
}

void SimpleController::FillOutFlowKey(sw_flow_key &key, uint16_t src, uint16_t dst, uint32_t wildcards)
{
  NS_LOG_FUNCTION(this);

  key.wildcards = htonl(wildcards);
  
  key.flow.in_port = htons(-1);   // ignore, in_port = -1
  key.flow.dl_vlan = htons(OFP_VLAN_NONE);
  key.flow.dl_type = htons(ETH_TYPE_IP);
  // Mac48Address("00:00:00:00:00:00").CopyTo(key.flow.dl_src);  // source mac48 = "0.0.0.0.0.0"
  m_topo->m_macs[src].CopyTo(key.flow.dl_src);
  m_topo->m_macs[dst].CopyTo(key.flow.dl_dst);
  
  key.flow.nw_proto = -1;   // ignore, nw_proto = -1
  // key.flow.nw_src = htonl(0);    // source ipv4 = "0.0.0.0"
  key.flow.nw_src = htonl(m_topo->m_ips[src]);
  key.flow.nw_dst = htonl(m_topo->m_ips[dst]);
  key.flow.tp_src = htons(-1);    // ignore, source port = -1
  key.flow.tp_dst = htons(-1);    // ignore, destination port = -1
  
  key.flow.mpls_label1 = htonl (MPLS_INVALID_LABEL);    // For MPLS: Top of label stack
  key.flow.mpls_label2 = htonl (MPLS_INVALID_LABEL);
  key.flow.reserved = 0;    // not use
}

void SimpleController::SendFlowEntry(uint16_t sw, uint32_t wildcards, uint16_t src, uint16_t dst, uint16_t out_port)
{
  NS_LOG_FUNCTION(this);

  // Create matching key.
  sw_flow_key key;
  FillOutFlowKey(key, src, dst, wildcards);

  // Create output-to-port action
  ofp_action_output x[1];
  x[0].type = htons (OFPAT_OUTPUT);
  x[0].len = htons (sizeof(ofp_action_output));
  x[0].port = out_port;  // setup out_port

  // Create a new flow and setup on specified switch
  ofp_flow_mod* ofm = BuildFlow (key, -1, OFPFC_ADD, x, sizeof(x), OFP_FLOW_PERMANENT, OFP_FLOW_PERMANENT);
  SendToSwitch (m_swtches[sw], ofm, ofm->header.length);
  m_tcamNum[sw]--;
}

void SimpleController::StartDelayMeasure(std::map<uint16_t, std::set<uint16_t> > &solution)
{
  NS_LOG_FUNCTION(this);

  uint8_t edegNum = m_topo->m_edges.size() / 2;
  for (std::map<uint16_t, std::set<uint16_t> >::iterator it = solution.begin(); it != solution.end(); it++) {
    std::map<uint16_t, std::vector<uint16_t> > flows;  // map<node, vector<edge> >
    
    std::set<uint16_t> pre, next, temp;
    pre.insert(it->first + m_topo->m_numHost);
    temp = it->second;
    while (!temp.empty()) {
      for (std::set<uint16_t>::iterator iter = it->second.begin(); iter != it->second.end(); iter++) {
        Edge &edge = m_topo->m_edges[*iter];

        std::set<uint16_t>::iterator pos = pre.find(edge.src);
        if (pos != pre.end()) {
          flows[edge.src].push_back(*iter);
          next.insert(edge.dst);
          temp.erase(*iter);
          continue;
        }
        
        pos = pre.find(edge.dst);
        if (pos != pre.end()) {
          if (*iter > edegNum)
            flows[edge.dst].push_back(*iter - edegNum);
          else
            flows[edge.dst].push_back(*iter + edegNum);
          next.insert(edge.src);
          temp.erase(*iter);
        }
      }
      pre = next;
      it->second = temp;
    }

    InstallMonitor(it->first);
    SendProbeEntry(it->first, flows);
  }
}

void SimpleController::InstallMonitor(uint16_t node)
{
  NS_LOG_FUNCTION(this << node);

  probe_control_info* pci = (probe_control_info*)malloc(sizeof(probe_control_info));
  pci->header.version = OFP_VERSION;
  pci->header.type = OFPT_HELLO;
  pci->header.length = htons (sizeof(probe_control_info));
  pci->header.xid = 0;
  pci->type = PROBE_MONITOR;
  pci->monitor = node;

  SendToSwitch(m_swtches[pci->monitor], pci, pci->header.length);
}

void SimpleController::SendProbeEntry(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows)
{ 
  NS_LOG_FUNCTION(this);

  for (std::map<uint16_t, std::vector<uint16_t> >::iterator it = flows.begin(); it != flows.end() ; ++it) {
    uint16_t  num = it->second.size();
    
    uint16_t len = sizeof(probe_control_info) + sizeof(uint16_t) * num;
    probe_control_info* pci_f = (probe_control_info*)malloc(len);   // forward flow
    pci_f->header.version = OFP_VERSION;
    pci_f->header.type = OFPT_HELLO;
    pci_f->header.length = htons (len);
    pci_f->header.xid = 0;
    pci_f->type = PROBE_FLOW;
    pci_f->flag = 0;    // forward: 0; back: 1
    pci_f->monitor = monitor;

    for (uint16_t i = 0; i < num; ++i) {
      uint16_t length = sizeof(probe_control_info) + sizeof(uint16_t);
      probe_control_info* pci_b = (probe_control_info*)malloc(length);  // back flow
      pci_b->header.version = OFP_VERSION;
      pci_b->header.type = OFPT_HELLO;
      pci_b->header.length = htons (length);
      pci_b->header.xid = 0;
      pci_b->type = PROBE_FLOW;
      pci_b->flag = 1;    // forward: 0; back: 1
      pci_b->monitor = monitor;
      pci_b->out_port[0] = m_topo->m_edges[it->second[i]].dpt;
      SendToSwitch(m_swtches[m_topo->m_edges[it->second[i]].dst - m_topo->m_numHost], pci_b, pci_b->header.length);   

      // for src node to distribute probe copy to other node
      pci_f->out_port[i] = m_topo->m_edges[it->second[i]].spt;
    }
    
    // for src node to distribute probe copy to other node
    SendToSwitch(m_swtches[it->first - m_topo->m_numHost], pci_f, pci_f->header.length);
  }
}


void SimpleController::StartLoadBanlance(void)
{
  NS_LOG_FUNCTION(this);

  // get all flow
  std::ifstream fin("scratch/default-flow.txt");
  if (fin.good()) {
    uint32_t n;
    fin >> n;
    Flow_t flow;
    for (uint32_t i = 0; i < n; ++i) {
      fin >> flow.src >> flow.dst >> flow.port >> flow.interval >> flow.utili;
      flow.idx = i;
      m_flows.push_back(flow);
    }
  }
  else {
    NS_ASSERT_MSG(false, "open default-flow.txt failed");
  }
  /*for (uint16_t i = 0; i < m_flows.size(); ++i) {
    std::cout << m_flows[i].src << " " << m_flows[i].dst << " " << m_flows[i].port << " "
              << m_flows[i].interval << " " << m_flows[i].utili << std::endl;
  }*/

  // get all default path
  std::vector<Path_t> paths(m_topo->m_numHost);
  for (uint16_t i = 0; i < m_topo->m_numHost; ++i) {
    m_curPath.push_back(paths);
  }
  for (uint16_t src = 0; src < m_topo->m_numHost; ++src) {
    for (uint16_t dst = 0; dst < m_topo->m_numHost; ++dst) {
      if (src == dst) continue;
      uint16_t cur = src;
      while (cur != dst) {
        Path_t path = m_topo->Dijkstra(cur, dst);        
        m_curPath[src][dst].push_back(path[0]);
        cur = m_topo->m_edges[path[0]].dst;  // change current node
      }
    }
  }
  /*for (uint16_t src = 0; src < m_topo->m_numHost; ++src) {
    for (uint16_t dst = 0; dst < m_topo->m_numHost; ++dst) {
      std::cout << "host " << src << "--->" << dst << ": ";
      Path_t &path = m_curPath[src][dst];
      for (uint16_t i = 0; i < path.size(); ++i) {
        Edge &edge = m_topo->m_edges[path[i]];
        std::cout << "<" << edge.src << "," << edge.dst << "> ";
      }
      std::cout << std::endl;
    }
  }*/

  Simulator::Schedule (BANLANCE_PERIOD, &SimpleController::LoadBanlanceCalculate, this);
}

void SimpleController::LoadBanlanceCalculate(void)
{
  NS_LOG_FUNCTION(this);

  // use copy to calculate
  m_utilization_copy = m_utilization;

  while (1) {
    std::cout << "#########################################" << std::endl;
    std::cout << "At time " << Simulator::Now().GetSeconds() << "s load banlance calculate begin" << std::endl;
    
    // find links that more than threshold, and sort from large to small
    std::vector<uint16_t> links = FindAllNotGoodLink();
    if (links.empty()) break;
  
    // handle the link with max utilization
    std::vector<Flow_t> flows = GetAllFlowsOnLink(links[0]);  // find all the flows on this link, and sort from large to small

    // handle flow from large to small
    uint16_t idx;
    for (idx = 0; idx < flows.size(); ++idx) {
      // flow
      Flow_t &flow = flows[idx];
      std::cout << "*******************************" << std::endl;
      std::cout << "handle the flow: " << flow.src << "--->" << flow.dst << " " << flow.utili << std::endl;
    
      // old path
      Path_t &oldPath = m_curPath[flow.src][flow.dst];
      std::cout << "oldPath: ";
      for (uint16_t i = 0; i < oldPath.size(); ++i) {
        Edge &edge = m_topo->m_edges[oldPath[i]];
        std::cout << "<" << edge.src << "," << edge.dst << "> ";
      }
      std::cout << std::endl;

      // find a new path
      Path_t newPath = GetNewPathWithoutSomeLink(flow.src, flow.dst, links);
      if (newPath.empty()) {
        std::cout << "Can not find a new path, continue\n";
        continue;
      }

      // store origin utilization
      std::vector<float> utili_origin = m_utilization_copy;

      // check link utilization
      if (!UtilizationCheck(oldPath, newPath, flow.utili)) {
        m_utilization_copy = utili_origin;
        std::cout << "Utilization check failed, continue\n";
        continue;
      }

      // check TCAM number
      std::map<uint16_t, uint16_t> sw_port;
      if(TcamCheck(oldPath, newPath, sw_port)) {
        m_curPath[flow.src][flow.dst] = newPath;
        UpdateFlow(flow.src, flow.dst, sw_port);
        for (uint16_t i = 0; i < m_tcamNum.size(); ++i) {
          m_tcam_file << m_tcamNum[i] << " ";
        }
        m_tcam_file << std::endl;
        break;
      }
      else {
        m_utilization_copy = utili_origin;
        std::cout << "Tcam check falied, continue\n";
      }
    }
    if (idx == flows.size())  break;  // have no flow can move
  }

  if (Simulator::Now() + BANLANCE_PERIOD < m_topo->m_simuTime)
    Simulator::Schedule (BANLANCE_PERIOD, &SimpleController::LoadBanlanceCalculate, this);
}

std::vector<uint16_t> SimpleController::FindAllNotGoodLink(void)
{
  NS_LOG_FUNCTION(this);

  // get all links that more than threshold
  std::vector<uint16_t> links;
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    if (m_utilization_copy[i] > LINK_THRESHOLD)
      links.push_back(i);
  }

  // sort from large to small
  for (uint16_t i = 0; i < links.size(); ++i) {
    float max = m_utilization_copy[links[i]];
    uint16_t index = i;
    for (uint16_t j = i + 1; j < links.size(); ++j) {
      if (m_utilization_copy[links[j]] > max) {
        max = m_utilization_copy[links[j]];
        index = j;
      }
    }
    uint16_t temp = links[i];
    links[i] = links[index];
    links[index] = temp;
  }

  // show
  for (uint16_t i = 0; i < links.size(); ++i) {
    std::cout << "congestion link <" << m_topo->m_edges[links[i]].src << "," 
              << m_topo->m_edges[links[i]].dst << "> " << m_utilization_copy[links[i]] << std::endl;
  }
  return links;
}

bool flow_compare(Flow_t a, Flow_t b) { return a.utili > b.utili; }

std::vector<Flow_t> SimpleController::GetAllFlowsOnLink(uint16_t link)
{
  NS_LOG_FUNCTION(this);

  std::vector<Flow_t> flows;
  for (uint16_t i = 0; i < m_flows.size(); ++i) {
    Flow_t &flow = m_flows[i];
    Path_t &path = m_curPath[flow.src][flow.dst];
    for (uint16_t j = 1; j < path.size() - 1; ++j) {
      if (path[j] == link || path[j] == link + m_edgeNum) {
        flows.push_back(flow);
      }
    }
  }
  std::sort(flows.begin(), flows.end(), flow_compare);

  // show
  std::cout << "there are " << flows.size() << " flows on the most congested link\n";
  for (uint16_t i = 0; i < flows.size(); ++i) {
    std::cout << flows[i].src << "--->"<< flows[i].dst << ": " << flows[i].utili << "\n";
  }
  return flows;
}

Path_t SimpleController::GetNewPathWithoutSomeLink(uint16_t src, uint16_t dst, const std::vector<uint16_t> &links)
{
  NS_LOG_FUNCTION(this);

  // increase distance on all congestion links
  for (uint16_t i = 0; i < links.size(); ++i) {
    m_topo->m_edges[links[i]].dist = 1000;
    m_topo->m_edges[links[i] + m_edgeNum].dist = 1000;
  }

  // get new path 
  Path_t newPath = m_topo->Dijkstra(src, dst);
  // check
  bool flag = false;
  for (uint16_t i = 0; i < newPath.size(); ++i) {
    for (uint16_t j = 0; j < links.size(); ++j) {
      if (newPath[i] == links[j] || newPath[i] == links[j] + m_edgeNum) {
        newPath.clear();
        flag = true;
        break;
      }
    }
    if (flag) break;
  }

  // return distance on all congestion links
  for (uint16_t i = 0; i < links.size(); ++i) {
    m_topo->m_edges[links[i]].dist = 1;
    m_topo->m_edges[links[i] + m_edgeNum].dist = 1;
  }

  // show
  std::cout << "newPath: ";
  for (uint16_t i = 0; i < newPath.size(); ++i) {
    Edge &edge = m_topo->m_edges[newPath[i]];
    std::cout << "<" << edge.src << "," << edge.dst << "> ";
  }
  std::cout << std::endl;

  return newPath;
}

bool SimpleController::UtilizationCheck(const Path_t &oldPath, const Path_t &newPath, double demand)
{
  NS_LOG_FUNCTION(this);
  
  // get max utilization
  double max = 0;
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    if (max < m_utilization_copy[i])
      max = m_utilization_copy[i];
  }

  // for old path
  for (uint16_t i = 1; i < oldPath.size() - 1; ++i) {
    if (oldPath[i] < m_edgeNum)
      m_utilization_copy[oldPath[i]] -= demand;
    else
      m_utilization_copy[oldPath[i] - m_edgeNum] -= demand;
  }
  // for new path
  for (uint16_t i = 1; i < newPath.size() - 1; ++i) {
    if (newPath[i] < m_edgeNum)
      m_utilization_copy[newPath[i]] += demand;
    else
      m_utilization_copy[newPath[i] - m_edgeNum] += demand;
  }

  // get new max utilization
  double max_new = 0;
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    if (max_new < m_utilization_copy[i])
      max_new = m_utilization_copy[i];
  }

  std::cout << "max new " << max_new << std::endl;
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    if (m_utilization_copy[i] > LINK_THRESHOLD)
      std::cout << "link <" << m_topo->m_edges[i].src << "," << m_topo->m_edges[i].dst << "> " << m_utilization_copy[i] << std::endl;
  }

  return max_new < max;
}
 
bool SimpleController::TcamCheck(const Path_t &oldPath, const Path_t &newPath, std::map<uint16_t, uint16_t> &sw_port)
{
  NS_LOG_FUNCTION(this);

  std::vector<Edge> &edges = m_topo->m_edges;

  // std::map<uint16_t, uint16_t> sw_port;
  for (uint16_t i = 1; i < newPath.size(); ++i) {
    uint16_t sw = edges[newPath[i]].src;
    
    // find in old path
    bool flag = true;
    for (uint16_t j = 1; j < oldPath.size() - 1; ++j){
      if (sw == edges[oldPath[j]].src && newPath[i] != oldPath[j]) {
        sw_port[sw - m_topo->m_numHost] = edges[newPath[i]].spt;
        flag = false;
        break;
      }
    }

    // find in switch path
    if (flag) {
      Path_t &path = m_swPath[sw - m_topo->m_numHost][edges[oldPath.back()].dst];
      if (newPath[i] != path[0])
        sw_port[sw - m_topo->m_numHost] = edges[newPath[i]].spt;
    }
  }

  std::cout << "switch-out_port: ";
  for (std::map<uint16_t, uint16_t>::iterator it = sw_port.begin(); it != sw_port.end(); ++it) {
    std::cout << "(" << it->first << ", " << it->second << ") ";
  }
  std::cout << std::endl;

  for (std::map<uint16_t, uint16_t>::iterator it = sw_port.begin(); it != sw_port.end(); ++it) {
    if (m_tcamNum[it->first] < 1)
      return false;
  }
  return true;
}

void SimpleController::UpdateFlow(uint16_t src, uint16_t dst, std::map<uint16_t, uint16_t> &sw_port)
{
  NS_LOG_FUNCTION(this);

  for (std::map<uint16_t, uint16_t>::iterator it = sw_port.begin(); it != sw_port.end(); ++it) {
   std::cout << "send flow(" << src << "--->" << dst << ") entry to switch " << it->first << std::endl;
    SendFlowEntry(it->first, 0, src, dst, it->second);
  }
}

void SimpleController::ReceiveHello(ofpbuf* buffer)
{
  NS_LOG_FUNCTION(this);

  ofp_header *oh = (ofp_header*)ofpbuf_at_assert (buffer, 0, sizeof (ofp_header));
  switch(oh->xid) {
    case 0:
      NS_LOG_WARN("ReceiveHello xid = 0");
      break;
    case 1:
      ReceiveUtilization(buffer);
      break;
    case 2:
      ReceiveDelay(buffer);
      break;
    default:
      NS_LOG_ERROR ("Can't receive this report_info message!" << oh->xid);
  }
}


void SimpleController::ReceiveUtilization(ofpbuf* buffer)
{
  NS_LOG_FUNCTION(this);

  NS_ASSERT(sizeof(utilization_report_info) <= buffer->size);
  
  utilization_report_info *uri = (utilization_report_info*)ofpbuf_try_pull(buffer, sizeof(utilization_report_info));
  uri->sw += m_topo->m_numHost;
  uint16_t cnt = (uri->header.length - sizeof(utilization_report_info)) / sizeof(uint64_t);

  double temp = m_bandWidth / 1000 * UTILIZATION_PERIOD.GetMilliSeconds();
  for (uint16_t i = 0; i < m_edgeNum; ++i) {
    Edge &edge =  m_topo->m_edges[i];
    if (edge.src == uri->sw) {
      m_utilization[i] = uri->data[edge.spt] * 8 / temp;
      cnt--;
    }
    else if (edge.dst == uri->sw) {
      m_utilization[i] = uri->data[edge.dpt] * 8 / temp;
      cnt--;
    }

    if (cnt == 0) break;
  }
}

void SimpleController::ReceiveDelay(ofpbuf* buffer)
{
  NS_LOG_FUNCTION(this);

  NS_ASSERT(sizeof(probe_report_info) <= buffer->size);

  probe_report_info *pci = (probe_report_info*)ofpbuf_try_pull(buffer, sizeof(probe_report_info));
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    uint16_t src = m_topo->m_edges[i].src - m_topo->m_numHost;
    uint16_t dst = m_topo->m_edges[i].dst - m_topo->m_numHost;
    if ((src == pci->src && dst == pci->dst) || (src == pci->dst && dst == pci->src)) {
      m_rtt[i] = pci->rtt;
      break;
    }
  }
}

void SimpleController::ReceivePacketIn(ofpbuf* buffer)
{
	NS_LOG_FUNCTION(this);
  NS_LOG_WARN("ReceivePacketIn");
}

void SimpleController::ReceivePortStatus(ofpbuf* buffer)
{
	NS_LOG_FUNCTION(this);

	ofp_port_status *ops = (ofp_port_status*)ofpbuf_try_pull(buffer, sizeof (ofp_port_status));
	switch(ops->reason){
		case OFPPR_ADD:
			NS_LOG_INFO("Add new ports!");
			break;
		case OFPPR_DELETE:
			NS_LOG_INFO("Delete the ports!");
			break;
		case OFPPR_MODIFY:
			NS_LOG_INFO("Modify the ports!");
			break;
		default:
			NS_LOG_ERROR ("Can't receive this port status message!");
	}
}

void SimpleController::OutputFile(void)
{
  NS_LOG_FUNCTION(this);

  // for rtt
  for (uint16_t i = 0; i < m_rtt.size(); ++i) {
    m_rtt_file << Time(m_rtt[i]).GetNanoSeconds() << " ";
  }
  m_rtt_file << std::endl;

  // for utilization
  for (uint16_t i = 0; i < m_utilization.size(); ++i) {
     m_utilization_file << m_utilization[i] << " ";
   }
   m_utilization_file << std::endl;

  /* Output average utilization */
  // double total = 0.0;
  // for (uint16_t i = 0; i < m_utilization.size(); ++i) {
  //   total += m_utilization[i];
  // }
  // m_utilization_file << total / m_utilization.size() << std::endl;

  if (Simulator::Now() + OUTPUT_FILE_PERIOD < m_topo->m_simuTime)
    Simulator::Schedule (OUTPUT_FILE_PERIOD, &SimpleController::OutputFile, this);
}

}	// namespace ns3