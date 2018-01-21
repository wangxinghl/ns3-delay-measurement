/**
  * @author: wangxing
  * @date: 2018.01.15
  */
#include <fstream>
#include "ns3/log.h"
#include "ns3/internet-module.h"
#include "ns3/openflow-module.h"
#include "topology.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Topology");

NS_OBJECT_ENSURE_REGISTERED(Topology);

TypeId Topology::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Topology")
    .SetParent<Object> ()
    .SetGroupName ("Openflow")
  ;
  return tid;
}

Topology::Topology ()
{
  NS_LOG_FUNCTION (this);
} 

Topology::~Topology ()
{
  NS_LOG_FUNCTION (this);

  m_edges.clear ();
  m_macs.clear ();
  m_ips.clear ();
  m_hosts.clear ();
  m_switches.clear ();
  delete_2_array<int> (m_numSw + m_numHost, adj);   // Release the memory space

  // delete adjacent matrix
  for (std::vector<AdjacentNode>::iterator it = m_switchTopo.begin(); it != m_switchTopo.end(); it++) {
    AdjacentEdge *q, *p = it->firstEdge;
    while (p) {
      q = p->next;
      delete p;
      p = q;
    }
  }
}

void Topology::BuildTopo (const char* file, Time simuTime, std::string bandwidth)
{
  NS_LOG_FUNCTION (this);

  m_bps = bandwidth;

  // Read topology file
  std::ifstream fin (file);
  NS_ASSERT_MSG (fin.good (), "Open topology file failed!");
  ReadTopo (fin);

  // Create host node
  NodeContainer hostNodes; 
  hostNodes.Create (m_numHost);
  // Create switch node
  NodeContainer switchNodes;
  switchNodes.Create (m_numSw);

  // According to link information in topology file, create csma channel between host and switch, switch and switch
  std::vector<NetDeviceContainer> switchPorts;
  switchPorts = DoConnect (fin, bandwidth, hostNodes, switchNodes);
  fin.close ();

  // Install internet stack on host nodes and switch nodes
  InternetStackHelper internet;
  internet.Install (hostNodes);
  internet.Install (switchNodes);

  // Create a simple controller
  Ptr<SimpleController> control = Create<SimpleController> ();
  
  // Install openflow switch net device on switch nodes
  OpenFlowSwitchHelper swtch;
  for (uint16_t i = 0; i < m_numSw; i++) {
    swtch.SetDeviceAttribute ("ID", UintegerValue (i));  // Set switch id
    NetDeviceContainer ofDevice = swtch.Install (switchNodes.Get (i), switchPorts[i]);  // Install openflow switch net device
    
    Ptr<OpenFlowSwitchNetDevice> opendev = DynamicCast<OpenFlowSwitchNetDevice, NetDevice> (ofDevice.Get (0));
    opendev->SetController(control);
    opendev->SetSimuTime(simuTime);
    m_switches.push_back (opendev);  // Collect the pointer to switch
  }
  
  // Assign network to multiple subnet. Then collect ipv4 address and mac address of all nodes, and set arp cache
  AssignSubnet ();

  // Do some initialization work
  Init ();

  // Create adjacent matrix
  CreateAdjacentMatrix();
  control->SetTopology(this);

  // for (size_t i = 0; i < m_ips.size(); ++i) {
  //   std::cout << Ipv4Address(m_ips[i]) << std::endl;
  // }
}

Ptr<Node> Topology::GetNode (uint16_t nodeId)
{
  NS_LOG_FUNCTION (this);
  
  if (nodeId < m_numHost) {
    return m_hosts[nodeId]->GetNode ();   // Host node
  }
  else if (nodeId < m_numSw + m_numHost) {
    return m_switches[nodeId - m_numHost]->GetNode ();    // Switch node
  }
  else {
    NS_ASSERT_MSG (nodeId < m_numSw + m_numHost, "Topology don't have this node! the nodeID is: " <<(uint32_t) nodeId);
  }
  return NULL;
}

uint64_t Topology::GetBandwidth (void)
{
  NS_LOG_FUNCTION (this);
  return m_bps.GetBitRate ();
}

Path_t Topology::Dijkstra (uint16_t src, uint16_t dst)
{
  NS_LOG_FUNCTION (this);

  const int INF = std::numeric_limits<uint16_t>::max ();

  int dist[m_numSw + m_numHost];
  int parent[m_numSw + m_numHost];
  bool visit[m_numSw + m_numHost];
  // Initialize 
  for (int i = 0; i < m_numSw + m_numHost; ++i) {
    dist[i] = INF;
    parent[i] = -1;
    visit[i] = false;
  }
  dist[src] = 0;

  // Dijkstra
  for (int i = 0; i < m_numSw + m_numHost; ++i) {
    int k = -1;
    int min = INF;
    for (int j = 0; j < m_numSw + m_numHost; ++j) {
      if (!visit[j] && dist[j] < min) {
        min = dist[j];
        k = j;
      }
    }
    
    if (k == dst) break;
    if (k != -1) {
      visit[k] = true;
      for (int j = 0; j < m_numSw + m_numHost; ++j) {
        if (!visit[j] && adj[k][j] >= 0 && (m_edges[adj[k][j]].dist + dist[k] < dist[j])) {
          dist[j] = dist[k] + m_edges[adj[k][j]].dist;
          parent[j] = adj[k][j];
        }
      }
    }
  }

  // find the path (reverse path)
  Path_t path;
  int start = dst;
  while (parent[start] != -1) {
    path.push_back (parent[start]);
    start = m_edges[parent[start]].src;
  }
  return path;
}

uint16_t Topology::GetSwitchEdgeNum(void)
{
    uint16_t cnt = 0;
    for (size_t i = 0; i < m_edges.size(); ++i) {
        if (m_edges[i].ssFlag)
            cnt++;
    }
    return cnt / 2;
}

std::map<uint16_t, uint16_t> Topology::GetEdgeAdjacentNode(uint16_t edge, uint16_t depth)
{
    std::map<uint16_t, uint16_t> result;
    std::vector<std::set<uint16_t> > res(depth);
    // depth = 1
    uint16_t node = m_edges[edge].src - m_numHost;
    result[node] = 1;
    res[0].insert(node);
    
    node = m_edges[edge].dst - m_numHost;
    result[node] = 1;
    res[0].insert(node);
    
    // depth = other
    for (uint16_t i = 1; i < depth; i++) {
        std::set<uint16_t> &pre = res[i - 1];
        std::set<uint16_t> &now = res[i];
        for (std::set<uint16_t>::iterator it = pre.begin(); it != pre.end(); it++) {
            AdjacentEdge *p = m_switchTopo[*it].firstEdge;
            while (p) {
                if (result.find(p->dst) == result.end()) {
                    result[p->dst] = i + 1;
                    now.insert(p->dst);
                }
                p = p->next;
            }
        }
    }
    return result;
}

void Topology::Init (void)
{
  NS_LOG_FUNCTION (this);

  // Add the reversed edges of full duplex channel
  uint16_t size = m_edges.size ();
  Edge redge;
  redge.dist = 1;
  for (uint16_t i = 0; i < size; ++i) {
    redge.idx = size + i;
    redge.src = m_edges[i].dst;
    redge.dst = m_edges[i].src;
    redge.spt = m_edges[i].dpt;
    redge.dpt = m_edges[i].spt;
    redge.ssFlag = m_edges[i].ssFlag;
    m_edges.push_back (redge);
  }

  // Apply memory for adj and initialize it
  uint16_t nodeNum = m_numSw + m_numHost;
  adj = alloc_2_array<int> (nodeNum, nodeNum);    // Apply memory 
  for (uint16_t i = 0; i < nodeNum; ++i) {
    for (uint16_t j = 0; j < nodeNum; ++j) {
      adj[i][j] = -1;   // Set dafault value to -1. Note that this two nodes are not directly connected
    }
  }
  for (uint32_t i = 0; i < m_edges.size(); ++i) {
    uint16_t src = m_edges[i].src;
    uint16_t dst = m_edges[i].dst;
    adj[src][dst] = i;    // If two nodes are directly connected, set value to edge id
  }
}

void Topology::ReadTopo (std::ifstream &fin)
{
  NS_LOG_FUNCTION (this);

  std::string item;   // Read the topology layer
  fin >> item >> m_layer;
  if (m_layer == 2) {  // For 2-layer
    fin >> item >> m_numHost;       // Read the number of host
    fin >> item >> m_ctrlPos;       // Read the host id of controller
    fin >> item >> m_numSw_bottom;  // Read the number of bottom switch
    fin >> item >> m_numSw_top;     // Read the number of top switch
    m_numSw = m_numSw_bottom + m_numSw_top; // Calculate the total number of switch
  }
  else {   // For 3-layer
    fin >> item >> m_numCoreSw;     // Read the number of core switch
    fin >> item >> m_numPOD;        // Read the number of POD
    fin >> item >> m_numSwPOD;      // Read the number of switch each POD
    fin >> item >> m_numHost;       // Read the number of host
    fin >> item >> m_ctrlPos;       // Read the host id of controller
    m_numSw = m_numCoreSw + m_numSwPOD * m_numPOD;  // Calculate the total number of switch
  }
  NS_LOG_INFO ("Topo: layer " << m_layer << " numSw " << m_numSw << " numHost " << m_numHost << " Controller " << m_ctrlPos);
}

std::vector<NetDeviceContainer> Topology::DoConnect (std::ifstream &fin, std::string bandwidth, NodeContainer &hosts, NodeContainer &switches)
{
  NS_LOG_FUNCTION (this);

  // Set link rate, the queue of net device
  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue (bandwidth));
  //csma.SetQueue (queue, "Mode", StringValue ("QUEUE_MODE_PACKETS"), "MaxPackets", UintegerValue (UINT32_MAX));
  csma.SetDeviceAttribute ("EncapsulationMode", StringValue ("Dix"));

  std::vector<NetDeviceContainer> switchPorts;    // Switch ports container vector
  switchPorts.resize (m_numSw);

  Edge edge;
  edge.dist = 1;      // The distance to another node

  uint16_t number, src, dst;
  while (fin >> number >> src >> dst) {
    edge.idx = number;  // The edge id
    edge.src = src;     // Soure node id
    edge.dst = dst;     // Destination node id

    edge.ssFlag = false;
    if (src >= m_numHost && dst >= m_numHost) {
      edge.ssFlag = true;   // The edge is between switcg and switch
    }

    NetDeviceContainer link;
    if (src >= m_numHost) {  // The edge is between switch and switch
      // Source switch --> Destination switch
      src = src - m_numHost;
      dst = dst - m_numHost;
      link = csma.Install (NodeContainer (switches.Get (src), switches.Get (dst)));
      switchPorts[src].Add (link.Get (0));  // Add switch ports
    }
    else {   // The edge is between host and switch
      // Source host --> Destination switch
      dst = dst - m_numHost;
      link = csma.Install (NodeContainer (hosts.Get (src), switches.Get (dst)));
      m_hosts.push_back (DynamicCast<CsmaNetDevice, NetDevice> (link.Get (0)));   // Store CsmaNetDevice point of host
    }
    switchPorts[dst].Add (link.Get (1));    // Add switch ports
    edge.spt = link.Get (0)->GetIfIndex ();   // Source port
    edge.dpt = link.Get (1)->GetIfIndex ();   // Destination port

    m_edges.push_back (edge);    // Store the edge
  }
  return switchPorts;
}

void Topology::AssignSubnet (void)
{
  NS_LOG_FUNCTION (this);

  std::vector<NetDeviceContainer> subnets;      // Net device contianer vector of subnet
  std::vector<Ipv4InterfaceContainer> interfaces;   // Ipv4 interface contianer vector of subnet
  std::map<uint16_t, Ptr<Ipv4> > map;   // The map of <nodeId, Ptr<Ipv4> >
  
  if (m_layer == 2) {  // For 2-layer
    uint16_t num = m_numHost / m_numSw_bottom;   // The number of host connecting to switch 

    subnets.resize(m_numSw_bottom + 1);   // The number of subnet is 'm_numSw_bottom + 1'
    interfaces.resize (m_numSw_bottom + 1);
    
    /**
     * Divide subnet
     *
     * Each bottom switch and its hosts is a subnet, and all the top layer switches is a subnet
     */
    for (uint16_t i = 0; i < m_numSw_top; ++i) {
      subnets[m_numSw_bottom].Add (m_switches[m_numSw_bottom + i]);
    }
    for (uint16_t i = 0; i < m_numSw_bottom; ++i) {
      subnets[i].Add (m_switches[i]);
    }
    for (uint16_t i = 0; i < m_numHost; ++i) {
      subnets[i / num].Add (m_hosts[i]);
    }

    /**
     * Assign ipv4 address
     *
     * Ipv4Address: 0.0.1.1 ---> uint32_t: 0x00000101
     * uint32_t: 0x00000101 + 0x00000100 * 1 ---> 0.0.2.1
     * uint32_t: 0x00000101 + 0x00000100 * 2 ---> 0.0.3.1
     * ......
     */
    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.0.0", "255.255.0.0");   // For top layer switch
    interfaces[m_numSw_bottom] = ipv4.Assign (subnets[m_numSw_bottom]);
    for (int i = 0; i < m_numSw_bottom; ++i) {   // For each bottom switch and its hosts
      ipv4.SetBase ("10.1.0.0", "255.255.0.0", Ipv4Address (0x00000101 + 0x00000100 * i));
      interfaces[i] = ipv4.Assign (subnets[i]);
    }

    // Collect ipv4 address of each node
    for (uint16_t i = 0; i < m_numSw_bottom; ++i) {   // For bottom layer switches and hosts
      uint16_t nodeId = m_numHost + i;
      for (uint32_t j = 0; j < subnets[i].GetN (); ++j) {
        if (j != 0) {
          nodeId = num * i + (j - 1);
        }
        m_ips[nodeId] = interfaces[i].GetAddress (j).Get ();  // Store the ipv4 address of specified node
        map[nodeId] = interfaces[i].Get (j).first;  // Store the Ptr<Ipv4> of specified node
      }
    }
    for (uint16_t i = 0; i < m_numSw_top; ++i) {    // For top layer switches
      m_ips[m_numHost + m_numSw_bottom + i] = interfaces[m_numSw_bottom].GetAddress (i).Get ();   // Store the ipv4 address of specified node
      map[m_numHost + m_numSw_bottom + i] = interfaces[m_numSw_bottom].Get (i).first;   // Store the Ptr<Ipv4> of specified node
    }
  }
  else {
    uint16_t half = m_numSwPOD / 2;   // The number of switch at each layer in one POD
    uint16_t half_pod = half * m_numPOD;    // The number of switch at each layer in all POD
    uint16_t num = m_numHost / m_numPOD / half;   // The number of host connecting to switch

    subnets.resize(half_pod + 1);   // The number of subnet is 'half_pod + 1'
    interfaces.resize (half_pod + 1);

    /**
     * Divide subnet
     *
     * Each bottom switch and its hosts is a subnet, and all the core switches and POD top layer switches is a subnet
     */
    for (uint16_t i = 0; i < m_numCoreSw; ++i) {  // Core switch layer
      subnets[half_pod].Add (m_switches[m_numSw - m_numCoreSw + i]);
    }
    for (uint16_t i = 0; i < m_numPOD; ++i) {   // POD layer
      for (uint16_t j = 0; j < half; ++j) {
        subnets[half_pod].Add (m_switches[half_pod + half * i + j]);  // POD top layer
        subnets[half * i + j].Add (m_switches[half * i + j]);   // POD bottom layer
      }
    }
    for (uint16_t i = 0; i < m_numHost; ++i) {  // Host layer
      subnets[i / num].Add (m_hosts[i]);
    }

    /**
     * Assign ipv4 address
     *
     * Ipv4Address: 0.0.1.1 ---> uint32_t: 0x00000101
     * uint32_t: 0x00000101 + 0x00000100 * 1 ---> 0.0.2.1
     * uint32_t: 0x00000101 + 0x00000100 * 2 ---> 0.0.3.1
     * ......
     */
    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    interfaces[half_pod] = ipv4.Assign (subnets[half_pod]);   // For core switches and POD top layer switches
    for (uint16_t i = 0; i < half_pod; ++i) {
      ipv4.SetBase ("10.1.0.0", "255.255.0.0", Ipv4Address (0x00000101 + 0x00000100 * i));
      interfaces[i] = ipv4.Assign (subnets[i]);
    }

    // Collect ipv4 address of node id
    for (uint16_t i = 0; i < half_pod; ++i) {
      uint16_t nodeId = m_numHost + i;
      for (uint32_t j = 0; j < interfaces[i].GetN (); ++j) {  // For POD bottom layer switches and hosts
        if (j != 0) {
          nodeId =  num * i + (j - 1);
        }
        m_ips[nodeId] = interfaces[i].GetAddress (j).Get ();    // Store the ipv4 address of specified node
        map[nodeId] = interfaces[i].Get (j).first;    // Store the Ptr<Ipv4> of specified node
      }
    }
    for (uint16_t i = 0; i < half_pod + m_numCoreSw; ++i) {  // For core switches and POD top layer switches
      uint16_t nodeId = m_numHost + half_pod + (i - m_numCoreSw);
      if (i < m_numCoreSw) {
        nodeId = m_numHost + m_numSw - m_numCoreSw + i;
      }
      m_ips[nodeId] = interfaces[half_pod].GetAddress (i).Get ();   // Store the ipv4 address of specified node
      map[nodeId] = interfaces[half_pod].Get (i).first;   // Store the Ptr<Ipv4> of specified node
    }
  }

  SetArpCache (map);  // Set arp cache of each node
}

void Topology::SetArpCache (std::map<uint16_t, Ptr<Ipv4> > &map)
{
  NS_LOG_FUNCTION (this);

  // Set arp cache between host and host
  for (uint16_t i = 0; i < m_numHost; ++i) {
    Mac48Address mac = Mac48Address::ConvertFrom (m_hosts[i]->GetAddress ());
    m_macs.push_back (mac);   // Store mac address of host

    for (uint16_t j = i + 1; j < m_numHost; ++j) {
      AddArpCache (map[i], m_ips[j], m_hosts[j]->GetAddress ());  // Src-->Dst
      AddArpCache (map[j], m_ips[i], mac);  // Dst-->Src
    }
  }

  // Set arp cache between switch and controller
  for (uint16_t i = 0; i < m_numSw; ++i) {
    Mac48Address mac = Mac48Address::ConvertFrom (m_switches[i]->GetAddress ());
    m_macs.push_back (mac);    // Store mac address of switch

    AddArpCache (map[m_ctrlPos], m_ips[m_numHost + i], mac);  // Controller-->Switch
    AddArpCache (map[m_numHost + i], m_ips[m_ctrlPos], m_hosts[m_ctrlPos]->GetAddress ());  // Switch-->Controller
  }
}

void Topology::AddArpCache (Ptr<Ipv4> ipv4, uint32_t ip, Address mac)
{
  NS_LOG_FUNCTION (this);

  Ptr<Ipv4L3Protocol> ipv4L3 = DynamicCast <Ipv4L3Protocol, Ipv4> (ipv4);
  Ptr<Ipv4Interface> interface = ipv4L3->GetInterface (1);    // Get Ipv4Interface
  Ptr<ArpCache> cache = interface->GetArpCache ();    // Get arp cache list

  ArpCache::Entry *entry = cache->Add (Ipv4Address (ip));   // Add arp cache entry
  entry->MarkAlive (mac);
  entry->MarkPermanent ();
}

void Topology::CreateAdjacentMatrix(void)
{
    NS_LOG_FUNCTION(this);
    m_switchTopo.resize(m_numSw);
    for (int i = 0; i < m_numSw; i++) {
        m_switchTopo[i].idx = i;
        m_switchTopo[i].firstEdge = NULL;
    }

    AdjacentEdge *p, *q;
    for (size_t i = 0; i < m_edges.size(); ++i) { 
        Edge &edge = m_edges[i];
        if (edge.ssFlag) {
            q = new AdjacentEdge;
            q->idx = edge.idx;
            q->dst = edge.dst - m_numHost;
            q->next = NULL;

            p = m_switchTopo[edge.src - m_numHost].firstEdge;
            if (p) {
                while (p->next) p = p->next;    // find the last edge
                p  ->next = q;
            }
            else {
                m_switchTopo[edge.src - m_numHost].firstEdge = q;
            }
        }
    }

    // for (std::vector<AdjacentNode>::iterator it = m_switchTopo.begin(); it != m_switchTopo.end(); it++) {
    //     std::cout << it->idx;
    //     AdjacentEdge *p = it->firstEdge;
    //     while (p) {
    //         std::cout << "--->" << p->dst << "(" << p->idx << ")";
    //         p = p->next;
    //     }
    //     std::cout << std::endl;
    // }
}

} // namespace ns3
