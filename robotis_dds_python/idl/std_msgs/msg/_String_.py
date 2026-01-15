"""
  Generated manually (matching Eclipse Cyclone DDS idlc Python Backend format)
  Module: std_msgs.msg
  IDL file: String_.idl
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class String_(idl.IdlStruct, typename="std_msgs.msg.dds_.String_"):
    data: str
