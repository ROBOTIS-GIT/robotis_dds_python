#!/usr/bin/env python3
"""Kill service Request message."""

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class Kill_Request(
    idl.IdlStruct,
    typename='physical_ai_interfaces::srv::dds_::Kill_Request_'
):
    """Kill service Request."""

    dummy: str = ''
