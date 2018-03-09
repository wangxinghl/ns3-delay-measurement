/**
 * @author: wangxing
 * @date: 2018.01.15
 */

#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "ns3/csma-module.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/openflow-switch-net-device.h"
#include "ns3/common.h"

namespace ns3 {

class Mac48Address;
class SimpleController;

class Topology : public Object
{

public:
  static TypeId GetTypeId(void);

  Topology ();
  virtual ~Topology ();
	
  void BuildTopo (const char* file, Time simuTime, std::string bandwidth = "1Gbps");
  
  Ptr<Node> GetNode (uint16_t nodeId);
  
  uint64_t GetBandwidth (void);

  Path_t Dijkstra (uint16_t src, uint16_t dst);

  uint16_t GetSwitchEdgeNum(void);
  std::map<uint16_t, uint16_t> GetEdgeAdjacentNode(uint16_t edge, uint16_t depth);

  uint16_t m_layer;        //!< The layer number of switch.
  uint16_t m_numSw;        //!< The total number of switch.
  uint16_t m_numHost;      //!< The total number of host, including controller.
  uint16_t m_ctrlPos;      //!< The node id of controller.
  
  uint16_t m_numSw_bottom; //!< The number of bottom layer switch (for 2-layer topology).
  uint16_t m_numSw_top;    //!< The number of top layer switch (for 2-layer topology).
  
  uint16_t m_numCoreSw;    //!< The number of core switch (for 3-layer topology).
  uint16_t m_numPOD;       //!< The number of POD (for 3-layer topology).
  uint16_t m_numSwPOD;     //!< The number of switch in each POD (for 3-layer topology).

  std::vector<Edge> m_edges;            //!< The all edge in network.

  Ptr<SimpleController> m_controller;    
  
  std::vector<Mac48Address> m_macs;     //!< Mac address of all node.
  std::map<uint16_t, uint32_t> m_ips;   //!< Ipv4 address of all node: <nodeId, ipv4>.

  Time m_simuTime;

private:
  void Init (void);

  void ReadTopo (std::ifstream &fin);

  std::vector<NetDeviceContainer> DoConnect (std::ifstream &fin, std::string bandwidth, NodeContainer &hosts, NodeContainer &switches);

  void AssignSubnet (void);

  void SetArpCache (std::map<uint16_t, Ptr<Ipv4> > &map);

  void AddArpCache (Ptr<Ipv4> ipv4, uint32_t ip, Address mac);

  void CreateAdjacentMatrix(void);

  /* The definition of adjacent edge */
  struct AdjacentEdge
  {
    uint16_t idx;
    uint16_t dst;
    AdjacentEdge *next;
  };

  /* The definition of node */
  struct  AdjacentNode {
    uint16_t idx;
    AdjacentEdge *firstEdge;
  };

  DataRate m_bps;          //!< The bandwidth of edge.

  std::vector<AdjacentNode> m_switchTopo;
  
  std::vector<Ptr<CsmaNetDevice> > m_hosts;               // The pointer to host net device vector
  std::vector<Ptr<OpenFlowSwitchNetDevice> > m_switches;  // The pointer to switch vector

  int **adj;          //!< The pointer to 2-dimensional array.
};

} // namespace ns3

#endif  /* TOPOLOGY_H */
