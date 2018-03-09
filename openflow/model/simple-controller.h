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

  void StartLoadBanlance(void);
  void LoadBanlanceCalculate(void);
  std::vector<uint16_t> FindAllNotGoodLink(void);
  std::vector<Flow_t> GetAllFlowsOnLink(uint16_t link);
  Path_t GetNewPathWithoutSomeLink(uint16_t src, uint16_t dst, const std::vector<uint16_t> &links);
  bool UtilizationCheck(const Path_t &oldPath, const Path_t &newPath, double demand);
  bool TcamCheck(const Path_t &oldPath, const Path_t &newPath);
  void SendPathFlow(const Path_t &oldPath, const Path_t &newPath, const Flow_t &flow);

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
  std::ofstream m_utilization_file;
  
  // for RTT
  std::vector<int64_t> m_rtt;
  std::ofstream m_rtt_file;
  
  // for load banlance
  std::vector<Flow_t> m_flows;
  std::vector<uint16_t> m_tcamNum;
  std::vector<std::vector<Path_t> > m_defaultPath;

};

}	// namespace ns3

#endif	/* SIMPLE_CONTROLLER_H */