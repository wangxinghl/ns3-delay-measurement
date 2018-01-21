/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "tag.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (Tag);

TypeId 
Tag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Tag")
    .SetParent<ObjectBase> ()
    .SetGroupName("Network")
  ;
  return tid;
}

/**********************wangxing added**********************/
// NS_LOG_COMPONENT_DEFINE ("ProbeTag");
NS_OBJECT_ENSURE_REGISTERED (ProbeTag);

TypeId ProbeTag::GetTypeId (void) {
  static TypeId tid = TypeId ("ns3::ProbeTag")
    .SetParent<Tag> ()
    .SetGroupName ("Network")
    .AddConstructor<ProbeTag> ()
  ;
  return tid;
}

TypeId ProbeTag::GetInstanceTypeId (void) const {
  return GetTypeId ();
}

uint32_t ProbeTag::GetSerializedSize (void) const {
  return 0;
}

void ProbeTag::Serialize (TagBuffer i) const {
}

void ProbeTag::Deserialize (TagBuffer i) {
}

void ProbeTag::Print (std::ostream &os) const {
  os << "ProbeTag";
}
/**********************wangxing added**********************/

} // namespace ns3
