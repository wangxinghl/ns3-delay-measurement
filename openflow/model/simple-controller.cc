/**
  * @author: wangxing
  * @date: 2018.01.15
  */

#include "ns3/log.h"
#include "max-flow.h"
#include "ksp-yen.h"
#include "simple-controller.h"

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
}

SimpleController::~SimpleController ()
{
	NS_LOG_FUNCTION(this);

  // for (Rtt_t::iterator it = m_rtt.begin(); it != m_rtt.end(); ++it) {
  //   for (std::map<uint16_t, int64_t>::iterator iter = it->second.begin(); iter != it->second.end(); iter++) {
  //     std::cout << "<" << it->first << "," << iter->first << "> " << Time(iter->second).GetMilliSeconds() << "\n";
  //   }
  // }

  m_swtches.clear();
  m_solution.clear();
  m_rtt.clear();

  delete_2_array<Paths_t> (m_topo->m_numHost, m_allPaths);
}

void SimpleController::SetTopology(Ptr<Topology> topo)
{
	NS_LOG_FUNCTION(this);
	m_topo = topo;

  // max-flow calculate
  // MaxFlow maxflow;
  // maxflow.Calculate(topo, 2, 5);
  // maxflow.Solution(m_solution);
  // // maxflow.ShowSolution();
  // SetProbeFlowEntry();

  // SetDataFlowEntry();

  m_allPaths = alloc_2_array<Paths_t> (topo->m_numHost, topo->m_numHost);
  KSPYen kspYen(topo, 3);
  kspYen.LoadKPaths(m_allPaths);
  for (uint16_t i = 0; i < topo->m_numHost; ++i) {
    for (uint16_t j = 0; j < topo->m_numHost; ++j) {
      std::cout << "host " << i << "--->" << j << ":";
      for (uint16_t k = 0; k < m_allPaths[i][j].size(); ++k) {
        std::cout << " (" << k << ") ";
        for (uint16_t kk = m_allPaths[i][j][k].size() - 1; kk > 0; --kk) {
          std::cout << m_topo->m_edges[m_allPaths[i][j][k][kk]].dst << " ";
        }
      }
      std::cout << std::endl;
    }
  }
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
        ReceiveDelay(buffer);
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
      for (uint16_t i = size - 1; i > 0; --i) {
        // get switch
        uint16_t sw = m_topo->m_edges[path[i]].dst - m_topo->m_numHost;
        // setup in_port
        key.flow.in_port = htons(m_topo->m_edges[path[i]].dpt);
        // setup out_port
        x[0].port = m_topo->m_edges[path[i - 1]].spt;

        // Create a new flow and setup on specified switch
        ofp_flow_mod* ofm = BuildFlow (key, -1, OFPFC_ADD, x, sizeof(x), OFP_FLOW_PERMANENT, OFP_FLOW_PERMANENT);
        SendToSwitch (m_swtches[sw], ofm, ofm->header.length);
      }
    }
  }
}

void SimpleController::SetProbeFlowEntry(void)
{
  NS_LOG_FUNCTION(this);

  uint8_t edegNum = m_topo->m_edges.size() / 2;
  for (std::map<uint16_t, std::set<uint16_t> >::iterator it = m_solution.begin(); it != m_solution.end(); it++) {
    std::map<uint16_t, std::vector<uint16_t> > flows;  // map<node, vector<edge> >
    std::set<uint16_t> pre, next, temp;
    pre.insert(it->first);
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
    SendProbeFlow(it->first - m_topo->m_numHost, flows);
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
  pci->period = Seconds(1).GetTimeStep();

  SendToSwitch(m_swtches[pci->monitor], pci, pci->header.length);
}

void SimpleController::SendProbeFlow(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows)
{ 
  NS_LOG_FUNCTION(this);

  InstallMonitor(monitor);
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

void SimpleController::ReceiveDelay(ofpbuf* buffer)
{
  NS_LOG_FUNCTION(this);
  
  probe_report_info *pci = (probe_report_info*)ofpbuf_try_pull(buffer, sizeof(probe_report_info));
  if (pci->src < pci->dst)
    m_rtt[pci->src][pci->dst] = pci->rtt;
  else
    m_rtt[pci->dst][pci->src] = pci->rtt;
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

}	// namespace ns3