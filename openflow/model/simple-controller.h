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
  // for flow entry
  void SetFlowEntry(void);
  void SetSwitchToHostFlowEntry(void);
  void FillOutFlowKey(sw_flow_key &key, uint16_t src, uint16_t dst, uint32_t wildcards);
  void SendFlowEntry(uint16_t sw, uint32_t wildcards, uint16_t src, uint16_t dst, uint16_t out_port);

  // for RTT measure
  void StartDelayMeasure(std::map<uint16_t, std::set<uint16_t> > &solution);
  void InstallMonitor(uint16_t node);
  void SendProbeEntry(uint16_t monitor, std::map<uint16_t, std::vector<uint16_t> > &flows);

  // for load banlance
  void StartLoadBanlance(void);
  void LoadBanlanceCalculate(void);
  std::vector<uint16_t> FindAllNotGoodLink(void);
  std::vector<Flow_t> GetAllFlowsOnLink(uint16_t link);
  Path_t GetNewPathWithoutSomeLink(uint16_t src, uint16_t dst, const std::vector<uint16_t> &links);
  bool UtilizationCheck(const Path_t &oldPath, const Path_t &newPath, double demand);
  bool TcamCheck(const Path_t &oldPath, const Path_t &newPath, std::map<uint16_t, uint16_t> &sw_port);
  void UpdateFlow(uint16_t src, uint16_t dst, std::map<uint16_t, uint16_t> &sw_port);

  // handle receive message
  void ReceiveHello(ofpbuf* buffer);
  void ReceiveUtilization(ofpbuf* buffer);
  void ReceiveDelay(ofpbuf* buffer);

	void ReceivePacketIn(ofpbuf* buffer);
	void ReceivePortStatus(ofpbuf* buffer);

  void OutputFile(void);


  // basic
	Ptr<Topology> m_topo;
  uint16_t m_edgeNum;
  uint16_t m_switchEdgeNum;
  uint64_t m_bandWidth;
  std::vector<Ptr<OpenFlowSwitchNetDevice> > m_swtches;

  // for utlization
  std::vector<float> m_utilization;
  std::vector<float> m_utilization_copy;
  std::ofstream m_utilization_file;
  
  // for RTT
  std::vector<int64_t> m_rtt;
  std::ofstream m_rtt_file;
  
  // for TCAM
  std::vector<uint16_t> m_tcamNum;
  std::ofstream m_tcam_file;
  // for load banlance
  std::vector<Flow_t> m_flows;
  std::vector<std::vector<Path_t> > m_curPath;  // m_curPath[host][host]
  std::vector<std::vector<Path_t> > m_swPath;   // m_swPath[switch][host]

};

}	// namespace ns3

#endif	/* SIMPLE_CONTROLLER_H */