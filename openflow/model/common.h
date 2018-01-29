/**
 * @author: wangxing
 * date: 2018.01.15
 */

#ifndef COMMON_H 
#define COMMON_H

#include <vector>
#include "openflow-interface.h"

namespace ns3 {

const int K = 3;

/**
 * The definition of edge.
 */
struct Edge
{
  uint16_t idx;   //!< Id value.
  uint16_t dist;  //!< The distance.
  uint16_t src;   //!< Source node.
  uint16_t dst;   //!< Destination node.
  uint16_t spt;   //!< Source port.
  uint16_t dpt;   //!< Destination port.
  bool ssFlag;    //!< Check whether the link is between switches.
                  //!< If the link is between switches, set it to true, otherwise false.
};

enum ControlType
 {
   PROBE_MONITOR,
   PROBE_FLOW
 };

struct probe_control_info
{
  ofp_header header;
  uint8_t type;
  uint8_t flag;         // forward: 0; back: 1
  uint16_t monitor;
  int64_t period;
  uint16_t out_port[0];
};

struct probe_report_info
{
  ofp_header header;
  uint16_t src;
  uint16_t dst;
  int64_t rtt;
};

/**
 * The defination of openflow flow entry.
 */
// struct FlowData
// {
//   uint32_t sw_ip;         //!< Switch ipv4 address.
//   uint16_t command;       //!< Command to the flow entry.
//   uint16_t in_port;       //!< In port.
//   uint16_t out_port;      //!< Out port.
//   uint8_t mac_src[6];     //!< Source mac address.
//   uint8_t mac_dst[6];     //!< Destination mac address.
//   uint32_t ip_src;        //!< Source ipv4 address.
//   uint32_t ip_dst;        //!< Destination ipv4 address.
//   uint8_t tos;            //!< Tos value.
// };
// const size_t FLOW_DATA = sizeof (FlowData);

// struct  NodeTos
// {
//   int src;
//   int dst;
//   int tos;
// };

/**
 * The definition of path, contains edge id
 */
typedef std::vector<uint16_t> Path_t;

/**
 * The definition of path container, contains multiple paths
 */
typedef std::vector<Path_t> Paths_t;

typedef std::map<uint16_t, std::map<uint16_t, int64_t> > Rtt_t;

/**
 * The templates to allocate or delete one-dimensional array, two-dimensional array and three-dimensional array.
 */
template<typename T>
T* alloc_array (int x)
{
  T *pointer = new T[x];
  return pointer;
}

template<typename T>
T** alloc_2_array (int x, int y)
{
  T **pointer = new T*[x];
  for (int i = 0; i < x; i++)
  {
    pointer[i] = new T[y];
  }
  return pointer;
}

template<typename T>
T*** alloc_3_array (int x, int y, int z)
{
  T ***pointer = new T **[x];
  for (int i = 0; i < x; i++)
  {
    pointer[i] = new T *[y];
    for (int j = 0; j < y; j++)
    {
      pointer[i][j] = new T[z];
    }
  }
  return pointer;
}

template<typename T>
void delete_array (T *p)
{
  delete []p;
}

template<typename T>
void delete_2_array (int x, T **p)
{
  for (int i = 0; i < x; i++)
  {
    delete []p[i];
  }
  delete []p;
}

template<typename T>
void delete_3_array (int x, int y, T ***p)
{
  for (int i = 0; i < x; i++)
  {
    for (int j = 0; j < y; j++)
    {
      delete[]p[i][j];
    }
    delete []p[i];
  }
  delete []p;
}

} // namespace ns3

#endif /* COMMON_H*/
