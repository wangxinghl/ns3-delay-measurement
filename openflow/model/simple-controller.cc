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
  
  m_defaultPath.clear();
  m_tcamNum.clear();
}

void SimpleController::SetTopology(Ptr<Topology> topo)
{
	NS_LOG_FUNCTION(this);
	m_topo = topo;
  m_edgeNum = m_topo->m_edges.size() / 2;
  m_switchEdgeNum = topo->GetSwitchEdgeNum();
  m_bandWidth = topo->GetBandwidth();

  // RTT and utilization
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    m_rtt.push_back(0);
  }
  m_utilization.resize(topo->m_edges.size() / 2);
  // TCAM
  for (uint16_t i = 0; i < topo->m_numSw; ++i) {
    m_tcamNum.push_back(TCAM_MAX);
  }

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
  // SetDataFlowEntry();
  SetSwitchToHostFlowEntry();
  for (uint16_t i = 0; i < m_tcamNum.size(); ++i) {
    std::cout << m_tcamNum[i] << " ";
  }
  std::cout << std::endl;

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

void SimpleController::SetDataFlowEntry(void)
{
  NS_LOG_FUNCTION(this);
  
  for (uint16_t src = 0; src < m_topo->m_numHost; ++src) {
    for (uint16_t dst = 0; dst < m_topo->m_numHost; ++dst) {
      if (src == dst) continue;
      // Create matching key.
      sw_flow_key key;
      key.wildcards = 0;
      // key.flow.in_port = htons();
      key.flow.dl_vlan = htons(OFP_VLAN_NONE);
      key.flow.dl_type = htons(ETH_TYPE_IP);
      m_topo->m_macs[src].CopyTo(key.flow.dl_src);
      m_topo->m_macs[dst].CopyTo(key.flow.dl_dst);
      key.flow.nw_proto = -1;
      key.flow.nw_src = htonl(m_topo->m_ips[src]);
      key.flow.nw_dst = htonl(m_topo->m_ips[dst]);
      key.flow.tp_src = htons(-1);
      key.flow.tp_dst = htons(-1);
      key.flow.mpls_label1 = htonl (MPLS_INVALID_LABEL);    // For MPLS: Top of label stack
      key.flow.mpls_label2 = htonl (MPLS_INVALID_LABEL);
      key.flow.reserved = 0;

      // Create output-to-port action
      ofp_action_output x[1];
      x[0].type = htons (OFPAT_OUTPUT);
      x[0].len = htons (sizeof(ofp_action_output));
      // x[0].port = out_port;

      Path_t path = m_topo->Dijkstra(src, dst);
      uint16_t size = path.size ();
      for (uint16_t i = 1; i < size; ++i) {
        // get switch
        uint16_t sw = m_topo->m_edges[path[i]].src - m_topo->m_numHost;
        // setup in_port
        key.flow.in_port = htons(m_topo->m_edges[path[i - 1]].dpt);
        // setup out_port
        x[0].port = m_topo->m_edges[path[i]].spt;

        // Create a new flow and setup on specified switch
        ofp_flow_mod* ofm = BuildFlow (key, -1, OFPFC_ADD, x, sizeof(x), OFP_FLOW_PERMANENT, OFP_FLOW_PERMANENT);
        SendToSwitch (m_swtches[sw], ofm, ofm->header.length);
      }
    }
  }
}

void SimpleController::SetSwitchToHostFlowEntry(void)
{
  NS_LOG_FUNCTION(this);

  for (uint16_t sw = 0; sw < m_topo->m_numSw; ++sw) {
    for (uint16_t host = 0; host < m_topo->m_numHost; ++host) {
      // Create matching key.
      sw_flow_key key;
      key.wildcards = 0;
      key.flow.in_port = htons(-1);     // in_port = -1
      key.flow.dl_vlan = htons(OFP_VLAN_NONE);
      key.flow.dl_type = htons(ETH_TYPE_IP);
      Mac48Address("00:00:00:00:00:00").CopyTo(key.flow.dl_src);  // source mac48 = "0.0.0.0.0.0"
      m_topo->m_macs[host].CopyTo(key.flow.dl_dst);
      key.flow.nw_proto = -1;
      key.flow.nw_src = htonl(0);       // source ipv4 = "0.0.0.0"
      key.flow.nw_dst = htonl(m_topo->m_ips[host]);
      key.flow.tp_src = htons(-1);
      key.flow.tp_dst = htons(-1);
      key.flow.mpls_label1 = htonl (MPLS_INVALID_LABEL);    // For MPLS: Top of label stack
      key.flow.mpls_label2 = htonl (MPLS_INVALID_LABEL);
      key.flow.reserved = 0;

      // Create output-to-port action
      ofp_action_output x[1];
      x[0].type = htons (OFPAT_OUTPUT);
      x[0].len = htons (sizeof(ofp_action_output));
      // x[0].port = out_port;

      Path_t path = m_topo->Dijkstra(sw + m_topo->m_numHost, host);
      x[0].port = m_topo->m_edges[path[0]].spt;     // setup out_port

      // Create a new flow and setup on specified switch
      ofp_flow_mod* ofm = BuildFlow (key, -1, OFPFC_ADD, x, sizeof(x), OFP_FLOW_PERMANENT, OFP_FLOW_PERMANENT);
      SendToSwitch (m_swtches[sw], ofm, ofm->header.length);
      m_tcamNum[sw]--;
    }
  }
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
    SendProbeFlow(it->first, flows);
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

void SimpleController::SendProbeFlow(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows)
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
    m_defaultPath.push_back(paths);
  }
  for (uint16_t src = 0; src < m_topo->m_numHost; ++src) {
    for (uint16_t dst = 0; dst < m_topo->m_numHost; ++dst) {
      if (src == dst) continue;
      uint16_t cur = src;
      while (cur != dst) {
        Path_t path = m_topo->Dijkstra(cur, dst);        
        m_defaultPath[src][dst].push_back(path[0]);
        cur = m_topo->m_edges[path[0]].dst;  // change current node
      }
    }
  }
  // for (uint16_t src = 0; src < m_topo->m_numHost; ++src) {
  //   for (uint16_t dst = 0; dst < m_topo->m_numHost; ++dst) {
  //     std::cout << "host " << src << "--->" << dst << ": ";
  //     Path_t &path = m_defaultPath[src][dst];
  //     for (uint16_t i = 0; i < path.size(); ++i) {
  //       Edge &edge = m_topo->m_edges[path[i]];
  //       std::cout << "<" << edge.src << "," << edge.dst << "> ";
  //     }
  //     std::cout << std::endl;
  //   }
  // }

  Simulator::Schedule (BANLANCE_PERIOD, &SimpleController::LoadBanlanceCalculate, this);
}

void SimpleController::LoadBanlanceCalculate(void)
{
  NS_LOG_FUNCTION(this);

  // find the links that more than threshold and sort from large to small
  std::vector<uint16_t> links = FindAllNotGoodLink();

  // handle link from large to small
  for (uint16_t linkId = 0; linkId < links.size(); ++linkId) {
    // find all the flows on this link, and sort from large to small
    uint16_t congestion_link = links[linkId];
    std::cout << "congestion link <" << m_topo->m_edges[congestion_link].src << "," << m_topo->m_edges[congestion_link].dst << "> " << m_utilization[congestion_link] << std::endl;
    std::vector<Flow_t> flows = GetAllFlowsOnLink(congestion_link);
    for (uint16_t i = 0; i < flows.size(); ++i) {
      std::cout << flows[i].src << "--->"<< flows[i].dst << ": " << flows[i].utili << "\n";
    }

    // handle flow from large to small
    for (uint16_t flow_id = 0; flow_id < flows.size(); ++flow_id) {
      // remove the flow from current path and modify utilization
      Flow_t &flow = flows[flow_id];
      std::cout << flow.src << "--->" << flow.dst << " " << flow.utili << std::endl;
      Path_t &oldPath = m_defaultPath[flow.src][flow.dst];
      std::cout << "oldPath: ";
      for (uint16_t i = 0; i < oldPath.size(); ++i) {
        Edge &edge = m_topo->m_edges[oldPath[i]];
        std::cout << "<" << edge.src << "," << edge.dst << "> ";
      }
      std::cout << std::endl;

      Path_t newPath = GetNewPathWithoutSomeLink(flow.src, flow.dst, links);
      if (newPath.empty()) continue;

      // check link utilization
      std::vector<float> utili_origin = m_utilization;    // store origin utilization
      if (!UtilizationCheck(oldPath, newPath, flow.utili)) {
        m_utilization = utili_origin;
        std::cout << "Utilization Check continue\n";
        continue;
      }

      // TCAM number
      if(!TcamCheck(oldPath, newPath)) {
        std::cout << "tcam Check continue\n";
        continue;
      }

      SendPathFlow(oldPath, newPath, flow);
      break;
    }
    break;
  }

  // if (Simulator::Now() + BANLANCE_PERIOD < m_topo->m_simuTime)
    // Simulator::Schedule (BANLANCE_PERIOD, &SimpleController::LoadBanlanceCalculate, this);
}

std::vector<uint16_t> SimpleController::FindAllNotGoodLink(void)
{
  NS_LOG_FUNCTION(this);

  // get all links that more than threshold
  std::vector<uint16_t> links;
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    if (m_utilization[i] > LINK_THRESHOLD)
      links.push_back(i);
  }

  // sort from large to small
  for (uint16_t i = 0; i < links.size(); ++i) {
    float max = m_utilization[links[i]];
    uint16_t index = i;
    for (uint16_t j = i + 1; j < links.size(); ++j) {
      if (m_utilization[links[j]] > max) {
        max = m_utilization[links[j]];
        index = j;
      }
    }
    uint16_t temp = links[i];
    links[i] = links[index];
    links[index] = temp;
  }

  // show
  for (uint16_t i = 0; i < links.size(); ++i) {
    std::cout << "links <" << m_topo->m_edges[links[i]].src << "," << m_topo->m_edges[links[i]].dst << "> " << m_utilization[links[i]] << std::endl;
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
    Path_t &path = m_defaultPath[flow.src][flow.dst];
    for (uint16_t j = 1; j < path.size() - 1; ++j) {
      if (path[j] == link || path[j] == link + m_edgeNum) {
        flows.push_back(flow);
      }
    }
  }
  std::sort(flows.begin(), flows.end(), flow_compare);
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
    if (max < m_utilization[i])
      max = m_utilization[i];
  }

  // for old path
  for (uint16_t i = 1; i < oldPath.size() - 1; ++i) {
    if (oldPath[i] < m_edgeNum)
      m_utilization[oldPath[i]] -= demand;
    else
      m_utilization[oldPath[i] - m_edgeNum] -= demand;
  }
  // for new path
  for (uint16_t i = 1; i < newPath.size() - 1; ++i) {
    if (newPath[i] < m_edgeNum)
      m_utilization[newPath[i]] += demand;
    else
      m_utilization[newPath[i] - m_edgeNum] += demand;
  }

  // get new max utilization
  double max_new = 0;
  for (uint16_t i = 0; i < m_switchEdgeNum; ++i) {
    if (max_new < m_utilization[i])
      max_new = m_utilization[i];
  }

  return max_new < max;
}

bool SimpleController::TcamCheck(const Path_t &oldPath, const Path_t &newPath)
{
  NS_LOG_FUNCTION(this);
  
  std::vector<uint16_t> switchs;;
  for (uint16_t i = 0; i < newPath.size(); ++i) {  // new path
    bool flag = false;
    for (uint16_t j = 0; j < oldPath.size(); ++j) {
      if (newPath[i] == oldPath[j]) {
        flag = true;
        break;
      }
    }
    if (flag)
      switchs.push_back(m_topo->m_edges[newPath[i]].src - m_topo->m_numHost);
  }

  

  for (uint16_t i = 0; i < switchs.size(); ++i) {
    if (m_tcamNum[switchs[i]] == 0)
      return false;
  }
  return true;
}

void SimpleController::SendPathFlow(const Path_t &oldPath, const Path_t &newPath, const Flow_t &flow)
{
  NS_LOG_FUNCTION(this);
  std::cout << "send path flow " << flow.src << "--->" << flow.dst << std::endl;
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
  std::cout << "ReceivePacketIn\n";
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