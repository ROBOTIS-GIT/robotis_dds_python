#!/usr/bin/env python3
"""Kill service Response message."""

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class Kill_Response(
    idl.IdlStruct,
    typename='physical_ai_interfaces::srv::dds_::Kill_Response_'
):
    """Kill service Response."""

    success: bool = False
    message: str = ''
