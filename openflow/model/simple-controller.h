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
  void SetSwitchToHostFlowEntry(void);

  void StartDelayMeasure(std::map<uint16_t, std::set<uint16_t> > &solution);
  void InstallMonitor(uint16_t node);
  void SendProbeFlow(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows);

  void ReceiveHello(ofpbuf* buffer);
  void ReceiveUtilization(ofpbuf* buffer);
  void ReceiveDelay(ofpbuf* buffer);

	void ReceivePacketIn(ofpbuf* buffer);
	void ReceivePortStatus(ofpbuf* buffer);

  void OutputFile(void);

	Ptr<Topology> m_topo;
  uint64_t m_bandWidth;

  std::vector<Ptr<OpenFlowSwitchNetDevice> > m_swtches;

  std::vector<int64_t> m_rtt;
  std::ofstream m_rtt_file;

  std::vector<float> m_utilization;
  std::ofstream m_utilization_file;

  std::vector<uint16_t> m_numLeftTcam;
};

}	// namespace ns3

#endif	/* SIMPLE_CONTROLLER_H */