/**
  * @author: wangxing
  * @date: 2018.01.15
  */

#include "ns3/log.h"
#include "max-flow.h"
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
}

void SimpleController::SetTopology(Ptr<Topology> topo)
{
	NS_LOG_FUNCTION(this);
	m_topo = topo;

  // max-flow calculate
  MaxFlow maxflow;
  maxflow.Calculate(topo, 2, 5);
  maxflow.Solution(m_solution);
  // maxflow.ShowSolution();
  // for (std::map<uint16_t, std::vector<uint16_t> >::iterator it = m_solution.begin(); it != m_solution.end(); it++) {
  //     std::cout << it->first << "(" << it->second.size() <<"): ";
  //     for (uint16_t i = 0; i < it->second.size(); ++i) {
  //         std::cout << it->second[i] << " ";
  //     }
  //     std::cout << "\n";
  // }

  SetFlowEntry();
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
    switch(type){
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
  pci->period = Seconds(0.1).GetTimeStep();

  SendToSwitch(m_swtches[pci->monitor], pci, pci->header.length);
}


void SimpleController::SendProbeFlow(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows)
{ 
  NS_LOG_FUNCTION(this);

  // std::cout << "\nmonitor " << monitor << "\n";
  InstallMonitor(monitor);
  for (std::map<uint16_t, std::vector<uint16_t> >::iterator it = flows.begin(); it != flows.end() ; ++it) {
    // std::cout << "node " << it->first << ": ";
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
      pci_b->header.length = htons (sizeof(probe_control_info));
      pci_b->header.xid = 0;

      pci_b->type = PROBE_FLOW;
      pci_b->flag = 1;    // forward: 0; back: 1
      pci_b->monitor = monitor;
      pci_b->out_port[0] = m_topo->m_edges[it->second[i]].dpt;

      SendToSwitch(m_swtches[m_topo->m_edges[it->second[i]].dst - m_topo->m_numHost], pci_b, pci_b->header.length);   

      // for src node to distribute probe copy to other node
      pci_f->out_port[i] = m_topo->m_edges[it->second[i]].spt;

      // Edge &edge = m_topo->m_edges[it->second[i]];
      // std::cout << "(" << pci_f->out_port[i] << ", <" << edge.src << "," << edge.dst
      //           << ">, " << pci_b->out_port[0] << "), ";
    }
    // std::cout << std::endl;
    
    // NS_ASSERT_MSG(false, sizeof(probe_control_info));
    // for src node to distribute probe copy to other node
    SendToSwitch(m_swtches[it->first - m_topo->m_numHost], pci_f, pci_f->header.length);   
  }
}

void SimpleController::ReceivePacketIn(ofpbuf* buffer)
{
	NS_LOG_FUNCTION(this);
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