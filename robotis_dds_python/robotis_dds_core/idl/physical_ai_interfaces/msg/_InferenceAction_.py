#!/usr/bin/env python3
"""Inference action message."""

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid("sequential")
class InferenceAction_(idl.IdlStruct,
    typename="physical_ai_interfaces::msg::dds_::InferenceAction_"):
    success: bool = False
    message: str = ""
    payload: str = ""   # 🔥 Base64 문자열로 변경
