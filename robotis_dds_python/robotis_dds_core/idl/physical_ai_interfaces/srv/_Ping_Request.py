#!/usr/bin/env python3
"""Ping service Request message."""

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class Ping_Request(
    idl.IdlStruct,
    typename='physical_ai_interfaces::srv::dds_::Ping_Request_'
):
    """Ping service Request."""

    # Ping에는 특별한 데이터 없음
    dummy: str = ''
