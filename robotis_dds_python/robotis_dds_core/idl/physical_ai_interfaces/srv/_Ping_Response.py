#!/usr/bin/env python3
"""Ping service Response message."""

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class Ping_Response(
    idl.IdlStruct,
    typename='physical_ai_interfaces::srv::dds_::Ping_Response_'
):
    """Ping service Response."""

    success: bool = False
    message: str = ''
