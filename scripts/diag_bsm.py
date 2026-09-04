#!/usr/bin/env python3
"""
On-device BSM-fault diagnostic for the Fisker Ocean port.

Subscribes to the CAN service (bus 0) and prints EVERY state change of:
  - ADAS_0x314 : ADAS_BSDSts                 (0=Off 1=Standby 2=Available 3=Active 4=Error)
  - ADAS_0x315 : ADAS_BSD_CID_{Le,Ri}DispReq (0=No_threat 1..3=threat 4=Error)
  - ADAS_0x31A : ADAS_BSM_ELKA_TelltaleReq   (0=Normal 1=Off 2=Inactive 3=Fault_Degraded 4=Intervention)
plus openpilot's engaged / controls_allowed state.

Run on the comma 3X while doing an engage → cruise → disengage cycle. Every
printed line is a timestamped state change so we can pin whether BSM flips
DURING engagement, AT the moment of disengage, or AFTER.

Usage:
    cd /data/openpilot
    uv run python scripts/diag_bsm.py

Ctrl-C to stop.
"""
import time

import cereal.messaging as messaging
from openpilot.selfdrive.pandad import can_capnp_to_list
from opendbc.can.parser import CANParser


BSDSTS = {0: "Off", 1: "Standby", 2: "Available", 3: "Active", 4: "Error"}
DISP   = {0: "None", 1: "Threat", 2: "Threat+Blinker", 3: "Critical", 4: "Error"}
TELL   = {0: "Normal", 1: "Off", 2: "Inactive", 3: "Fault_Degraded", 4: "Intervention"}


def main() -> None:
  cp = CANParser("fisker_ocean_adas",
    [("ADAS_0x314", 50), ("ADAS_0x315", 20), ("ADAS_0x31A", 20)],
    bus=0,
  )

  can_sock = messaging.sub_sock("can", conflate=False, timeout=100)
  sm = messaging.SubMaster(["carState", "controlsState"])

  prev: dict = {}
  t0 = time.monotonic()
  last_heartbeat = t0

  print(f"{'t(s)':>8}  {'eng':>5} {'act':>5}  {'BSDSts':>17}  "
        f"{'LeDisp':>20} {'RiDisp':>20}  {'Telltale':>22}")
  print("-" * 110)

  while True:
    raw = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    if not raw:
      continue
    cp.update_strings(raw)
    sm.update(0)

    bsds = int(cp.vl["ADAS_0x314"]["ADAS_BSDSts"])
    le   = int(cp.vl["ADAS_0x315"]["ADAS_BSD_CID_LeDispReq"])
    ri   = int(cp.vl["ADAS_0x315"]["ADAS_BSD_CID_RiDispReq"])
    tell = int(cp.vl["ADAS_0x31A"]["ADAS_BSM_ELKA_TelltaleReq"])
    eng  = bool(sm["controlsState"].enabled)
    act  = bool(sm["controlsState"].active)

    cur = (bsds, le, ri, tell, eng, act)
    now = time.monotonic()
    if cur != tuple(prev.values()) or (now - last_heartbeat) > 10:
      t = now - t0
      print(f"{t:>8.2f}  {int(eng):>5} {int(act):>5}  "
            f"{bsds}={BSDSTS.get(bsds,'?'):>13}  "
            f"{le}={DISP.get(le,'?'):>17} {ri}={DISP.get(ri,'?'):>17}  "
            f"{tell}={TELL.get(tell,'?'):>19}",
            flush=True)
      prev = {"bsds": bsds, "le": le, "ri": ri, "tell": tell, "eng": eng, "act": act}
      last_heartbeat = now


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("\nstopped.")
