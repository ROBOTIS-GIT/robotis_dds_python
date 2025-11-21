#!/usr/bin/env python3
"""GetDatasetInfo service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg._DatasetInfo_ import DatasetInfo_


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetDatasetInfo_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetDatasetInfo_Response_'):
    """GetDatasetInfo service Response."""

    dataset_info: DatasetInfo_
    success: bool = False
    message: str = ''
