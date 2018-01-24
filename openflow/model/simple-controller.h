/**
 * @author: wangxing
 * @date: 2018.01.15
 */
#ifndef SIMPLE_CONTROLLER_H
#define SIMPLE_CONTROLLER_H

#include "openflow-interface.h"
#include "topology.h"

namespace ns3 {

class SimpleController : public ofi::Controller
{
public:
  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);

  SimpleController ();
  ~SimpleController ();
  
  void SetTopology(Ptr<Topology> topo);
  void AddSwitch (Ptr<OpenFlowSwitchNetDevice> swtch);
  void ReceiveFromSwitch (Ptr<OpenFlowSwitchNetDevice> swtch, ofpbuf* buffer);

private:
  void SetDataFlowEntry(void);
  void SetFlowEntry(void);
  
  void InstallMonitor(uint16_t node);
  void SendProbeFlow(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows);

  void ReceiveDelay(ofpbuf* buffer);
	void ReceivePacketIn(ofpbuf* buffer);
	void ReceivePortStatus(ofpbuf* buffer);

	Ptr<Topology> m_topo;

  std::vector<Ptr<OpenFlowSwitchNetDevice> > m_swtches;

  std::map<uint16_t, std::set<uint16_t> > m_solution;

  Delay_t m_delay; // vector<src, vector<dst, delay> >
  Delay_t m_delayReal; // vector<src, vector<dst, delay> >
};

}	// namespace ns3

#endif	/* SIMPLE_CONTROLLER_H */