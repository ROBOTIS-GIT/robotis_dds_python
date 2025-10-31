#!/usr/bin/env python3
"""
DDS Example - Send velocity command
"""
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import time

def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("🚗 Sending cmd_vel commands...")

    try:
        for i in range(10):
            lin = 0.1 * (i % 3 - 1)  # -0.1, 0.0, 0.1 반복
            ang = 0.2 * ((i + 1) % 3 - 1)
            rds.send_cmd_vel(lin, ang)
            print(f"Sent cmd_vel → linear={lin:.2f}, angular={ang:.2f}")
            time.sleep(1.0)
    finally:
        rds.close()

if __name__ == "__main__":
    main()
