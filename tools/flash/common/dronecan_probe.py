#!/usr/bin/env python3
"""
Bench probe for the launchxl DroneCAN bridge: watch NodeStatus and actively query
uavcan.protocol.GetNodeInfo, decoding the response (name / sw version / vcs_commit /
unique_id) -- the first real-bus check of the GetNodeInfo path.

Uses pydronecan's own slcan driver, so NO root / slcand / SocketCAN setup is needed when
pointed at the serial adapter directly. A SocketCAN interface name (can0) also works if you
prefer to bring the link up yourself.

Prereqs:
  * Target running product.out built with a STATIC node id (no DNA allocator on a bare
    adapter bus): BOARD=launchxl_drv8305evm MOTOR=am_4116_kv450 NODE_ID=25 PRODUCT=1 bash build.sh
  * 1 Mbit bus (BOARD_CAN_BITRATE), 120 ohm termination, adapter = /dev/ttyACM2 (0483:5740).

Usage:
  python3 tools/flash/common/dronecan_probe.py [device=/dev/ttyACM2] [target_node=25] [our_node=127]
"""
import sys
import dronecan

device = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM2'
target = int(sys.argv[2]) if len(sys.argv) > 2 else 25
ournid = int(sys.argv[3]) if len(sys.argv) > 3 else 127

node = dronecan.make_node(device, node_id=ournid, bitrate=1000000)
seen = set()
state = {'requested': False, 'got': False}


def on_node_status(ev):
    nid = ev.transfer.source_node_id
    s = ev.message
    if nid not in seen:
        seen.add(nid)
        print("NodeStatus  node=%-3d uptime=%ds health=%d mode=%d vendor=0x%04X"
              % (nid, s.uptime_sec, s.health, s.mode, s.vendor_specific_status_code))
    if nid == target and not state['requested']:
        state['requested'] = True
        print("-> sending GetNodeInfo request to node %d ..." % target)
        node.request(dronecan.uavcan.protocol.GetNodeInfo.Request(), target, on_node_info)


def on_node_info(ev):
    if ev is None:
        print("GetNodeInfo: TIMEOUT (no response from node %d)" % target)
        state['requested'] = False   # allow a retry on the next NodeStatus
        return
    r = ev.response
    name = bytes(r.name).decode('ascii', 'replace')
    uid = ' '.join('%02X' % b for b in r.hardware_version.unique_id)
    print("\n==== GetNodeInfo response from node %d ====" % target)
    print("  name         : %s" % name)
    print("  sw version   : %d.%d  vcs_commit=0x%08X  opt_flags=0x%02X"
          % (r.software_version.major, r.software_version.minor,
             r.software_version.vcs_commit, r.software_version.optional_field_flags))
    print("  hw version   : %d.%d" % (r.hardware_version.major, r.hardware_version.minor))
    print("  unique_id    : %s" % uid)
    print("  status       : uptime=%ds health=%d" % (r.status.uptime_sec, r.status.health))
    print("===========================================\n")
    state['got'] = True


node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)
print("listening on %s for node %d (Ctrl-C to stop) ..." % (device, target))
try:
    while True:
        node.spin(1)
except KeyboardInterrupt:
    print("\nstopped.")
