/**
  * @author: wangxing
  * @date: 2018.01.15
  */

#include "ns3/log.h"
#include "max-flow.h"
#include "YenTopKShortestPathsAlg.h"
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

  m_rtt_file.open("scratch/output-file-rtt.txt");
  m_utilization_file.open("scratch/output-file-utilization.txt");
  Simulator::Schedule (OUTPUT_FILE_PERIOD, &SimpleController::OutputFile, this);
}

SimpleController::~SimpleController ()
{
	NS_LOG_FUNCTION(this);

  m_rtt_file.close();
  m_utilization_file.close();

  m_swtches.clear();
  
  m_rtt.clear();
  m_utilization.clear();
  
  m_numLeftTcam.clear();
}

void SimpleController::SetTopology(Ptr<Topology> topo)
{
	NS_LOG_FUNCTION(this);
	m_topo = topo;
  m_bandWidth = topo->GetBandwidth();

  // RTT and utilization
  uint16_t numEdge = topo->GetSwitchEdgeNum();
  for (uint16_t i = 0; i < numEdge; ++i) {
    m_rtt.push_back(0);
  }
  m_utilization.resize(topo->m_edges.size() / 2);
  
  // TCAM number
  for (uint16_t i = 0; i < topo->m_numSw; ++i) {
    m_numLeftTcam.push_back(100);
  }

  /**
   * max-flow calculate
   **/
  MaxFlow maxflow;
  maxflow.Calculate(topo, 2, 8);    // depth = 2, max = 8;
  std::map<uint16_t, std::set<uint16_t> > solution;
  maxflow.Solution(solution);
  // show result
  for (std::map<uint16_t, std::set<uint16_t> >::iterator iter = solution.begin(); iter != solution.end(); ++iter) {
    std::cout << "switch " << iter->first << "(" << iter->second.size() << "): ";
    for (std::set<uint16_t>::iterator it = iter->second.begin(); it != iter->second.end(); ++it) {
      Edge &edge = topo->m_edges[*it];
      std::cout << "<" << edge.src << "," << edge.dst << ">, ";
    }
    std::cout << std::endl;
  }
  StartDelayMeasure(solution);

  /**
   * Set data flow entry
   **/
  // SetDataFlowEntry();
  SetSwitchToHostFlowEntry();
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
      // setup out_port
      x[0].port = m_topo->m_edges[path.back()].spt;

      // Create a new flow and setup on specified switch
      ofp_flow_mod* ofm = BuildFlow (key, -1, OFPFC_ADD, x, sizeof(x), OFP_FLOW_PERMANENT, OFP_FLOW_PERMANENT);
      SendToSwitch (m_swtches[sw], ofm, ofm->header.length);
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

  uint16_t numEdge = m_topo->m_edges.size() / 2;
  double temp = m_bandWidth / 1000 * UTILIZATION_PERIOD.GetMilliSeconds();
  for (uint16_t i = 0; i < numEdge; ++i) {
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

  uint16_t edegNum_Sw = m_topo->GetSwitchEdgeNum();
  probe_report_info *pci = (probe_report_info*)ofpbuf_try_pull(buffer, sizeof(probe_report_info));
  for (uint16_t i = 0; i < edegNum_Sw; ++i) {
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
    m_rtt_file << Time(m_rtt[i]).GetMilliSeconds() << " ";
  }
  m_rtt_file << std::endl;

  // for utilization
  for (uint16_t i = 0; i < m_utilization.size(); ++i) {
    m_utilization_file << m_utilization[i] << " ";
  }
  m_utilization_file << std::endl;

  if (Simulator::Now() + OUTPUT_FILE_PERIOD < m_topo->m_simuTime)
    Simulator::Schedule (OUTPUT_FILE_PERIOD, &SimpleController::OutputFile, this);

}

}	// namespace ns3