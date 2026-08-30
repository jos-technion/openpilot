
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.fisker.values import (
  BUTTON_MAP,
  CANBUS,
  DBC,
  STEER_THRESHOLD,
)


ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# VCU_GearSig values from DBC VAL_ table
GEAR_MAP = {
  1: GearShifter.park,
  2: GearShifter.neutral,
  3: GearShifter.reverse,
  4: GearShifter.drive,
  6: GearShifter.eco,
  7: GearShifter.sport,
}


# Map from MFSS DBC signal name to openpilot ButtonType
_BUTTON_TYPE = {
  "mainCruise":   ButtonType.mainCruise,
  "setCruise":    ButtonType.setCruise,
  "accelCruise":  ButtonType.accelCruise,
  "decelCruise":  ButtonType.decelCruise,
}
BUTTON_SIGNAL_TO_TYPE = {sig: _BUTTON_TYPE[name] for sig, name in BUTTON_MAP.items()}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # The gateway mirrors the body/HMI signals (gear, pedal, doors, seatbelt,
    # blinkers, MFSS buttons) onto ADASBUS, so the port reads everything from
    # a single bus (Bus.pt = ADASBUS). No IBUS1 tap is required.
    self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    # SecOC sync state parsed from GW_Syn_All (0x20): the current Trip and Reset
    # counters (shared by all secured PDUs) and the sync MAC for key verification.
    self.secoc_trip = 0
    self.secoc_reset = 0
    self.secoc_sync_mac = b"\x00\x00\x00"
    self.secoc_sync_seen = False

    # Button state edge detection
    self._prev_button_state = {sig: 0 for sig in BUTTON_SIGNAL_TO_TYPE}

  def update(self, can_parsers) -> structs.CarState:
    cp_pt = can_parsers[Bus.pt]
    ret = structs.CarState()

    # ---- Speed (wheel speeds + cluster) ----------------------------------
    wsf = cp_pt.vl["ESP_0x115"]
    wsr = cp_pt.vl["ESP_0x116"]
    ret.wheelSpeeds.fl = wsf["ESP_WhlSpd_LF"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = wsf["ESP_WhlSpd_RF"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = wsr["ESP_WhlSpd_RL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = wsr["ESP_WhlSpd_RR"] * CV.KPH_TO_MS

    ret.vEgoRaw = cp_pt.vl["ESP_0x318"]["ESP_VehSpd"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # ICC_DispVehSpd is the number on the cluster, in the driver-selected unit
    # (ICC_DispVehSpdUnit VAL_: 0=KMH, 1=MPH). It MUST be converted with the matching
    # factor — on a US car (mph cluster) treating it as km/h reads ~1.609x low
    # (35 mph shows as 22). vEgoCluster is what the UI displays in preference to vEgo.
    icc = cp_pt.vl["ICC_0x531"]
    icc_to_ms = CV.MPH_TO_MS if icc["ICC_DispVehSpdUnit"] == 1 else CV.KPH_TO_MS
    ret.vEgoCluster = icc["ICC_DispVehSpd"] * icc_to_ms
    ret.standstill = ret.vEgoRaw < 0.01

    # ---- IMU ----
    yrs112 = cp_pt.vl["YRS_0x112"]
    ret.yawRate = yrs112["YRS_YawRate"] * CV.DEG_TO_RAD
    # YRS_LgtAcce is g; convert to m/s^2 via gravity
    ret.aEgo = cp_pt.vl["YRS_0x113"]["YRS_LgtAcce"] * 9.81

    # ---- Steering ----
    eps_ang = cp_pt.vl["EPS_0x1C2"]
    eps_tq = cp_pt.vl["EPS_0x1C4"]
    ret.steeringAngleDeg = eps_ang["EPS_SteerWhlAgSig"]
    # rate from numerical diff is handled by selfdrived; provide raw signal if available

    drvr_tq_dir = eps_tq["EPS_DrvrSteerTqDir"]   # 0=CCW (positive), 1=CW (negative)
    drvr_tq_mag = eps_tq["EPS_DrvrSteerTq"]      # 0..8 Nm
    ret.steeringTorque = drvr_tq_mag * (-1.0 if int(drvr_tq_dir) == 1 else 1.0)
    ret.steeringTorqueEps = eps_ang["EPS_AsscMotCrtTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    # ---- Pedals ----
    # VCU_0x214 is the gateway-mirrored VCU status on ADASBUS (SecOC-protected,
    # but the signal payload is cleartext). Carries gear, accel pedal %, brake.
    vcu = cp_pt.vl["VCU_0x214"]
    ret.gasPressed = vcu["VCU_APSPerc"] > 1.0    # > 1% pedal = pressed
    ret.brakePressed = (cp_pt.vl["ESP_0x318"]["ESP_BrkPedlSts"] == 1) or (vcu["VCU_BrkSig"] == 1)
    # Master cylinder pressure → 0..1 brake fraction (clamped)
    mcyl = cp_pt.vl["ESP_0x120"]["ESP_MstCylP"]
    mcyl_offs = cp_pt.vl["ESP_0x120"]["ESP_MstCylPOffs"]
    brake_bar = max(0.0, mcyl - mcyl_offs)
    ret.brake = min(1.0, brake_bar / 100.0)      # ~100 bar = full

    # ---- Gear ----
    gear_val = int(cp_pt.vl["VCU_0x214"]["VCU_GearSig"])
    ret.gearShifter = GEAR_MAP.get(gear_val, GearShifter.unknown)

    # ---- Doors / seatbelt ----
    doors = cp_pt.vl["BCM_0x343"]
    ret.doorOpen = bool(doors["BCM_DrFrntDoorSts"] or doors["BCM_PasFrntDoorSts"]
                        or doors["BCM_LeReDoorSts"] or doors["BCM_RiReDoorSts"])
    # ACU_BucSwtStFrntDrvr VAL_: 0=Buckled, 1=Not_Buckled, 2=Fault, 3=Invalid
    ret.seatbeltUnlatched = cp_pt.vl["ACU_0x159"]["ACU_BucSwtStFrntDrvr"] == 1

    # ---- Blinkers ----
    bcm335 = cp_pt.vl["BCM_0x335"]
    ret.leftBlinker = bool(bcm335["BCM_LeTrunLampOutpCmd"])
    ret.rightBlinker = bool(bcm335["BCM_RiTrunLampOutpCmd"])

    # ---- Cruise state (VCU basic cruise control) ----
    # openpilot intercepts the ADAS module, so its ACC frames (0x313/0x31C) are on the
    # isolated side (bus 2) and not on the bus we read. The car uses the VCU's basic
    # hold-speed cruise (VCU_Sts_CC_ICC, 0x358), which is gateway-sourced and native to
    # bus 0. Enum: 0=Off 1=Init 2=Standby 3=Active 4=Override 9/10=Fault (no standstill).
    # Set speed VCU_CcTrgSpdDisp is in the driver-selected unit (VCU_DispSpdUnit_CC VAL_:
    # 0=KMH,1=MPH); 255=no_display. Populate speedCluster too so the UI "MAX" box renders.
    vcu358 = cp_pt.vl["VCU_0x358"]
    cc_state = int(vcu358["VCU_Sts_CC_ICC"])
    cc_disp = vcu358["VCU_CcTrgSpdDisp"]
    cc_speed = 0.0 if cc_disp >= 255 else cc_disp * (CV.MPH_TO_MS if vcu358["VCU_DispSpdUnit_CC"] == 1 else CV.KPH_TO_MS)

    ret.cruiseState.enabled = cc_state in (3, 4)
    ret.cruiseState.available = cc_state not in (0, 1, 9, 10)
    ret.cruiseState.standstill = False
    ret.cruiseState.speed = cc_speed
    ret.cruiseState.speedCluster = cc_speed

    # ---- Buttons (MFSS) ----
    mfs = cp_pt.vl["MFS_0x514"]
    button_events = []
    for sig, btype in BUTTON_SIGNAL_TO_TYPE.items():
      cur = int(mfs[sig])
      prev = self._prev_button_state[sig]
      # treat any non-zero state as "pressed" for the first frame of the press
      if cur != 0 and prev == 0:
        be = structs.CarState.ButtonEvent(pressed=True, type=btype)
        button_events.append(be)
      elif cur == 0 and prev != 0:
        be = structs.CarState.ButtonEvent(pressed=False, type=btype)
        button_events.append(be)
      self._prev_button_state[sig] = cur
    ret.buttonEvents = button_events

    # ---- Faults ----
    ret.accFaulted = cc_state in (9, 10) or bool(cp_pt.vl["ESP_0x114"]["ESP_FltIndcn_AEB"])

    # ---- SecOC sync (GW_Syn_All 0x20) ----
    # Trip counter = 2 bytes BE, Reset counter = 3 bytes BE, MAC = 3 bytes.
    sync = cp_pt.vl["GW_Syn_All"]
    self.secoc_trip = (int(sync["Syn_TripCntrVal_Byte0_All"]) << 8) | int(sync["Syn_TripCntrVal_Byte1_All"])
    self.secoc_reset = ((int(sync["Syn_RstCntrVal_Byte0_All"]) << 16)
                        | (int(sync["Syn_RstCntrVal_Byte1_All"]) << 8)
                        | int(sync["Syn_RstCntrVal_Byte2_All"]))
    self.secoc_sync_mac = bytes([
      int(sync["Syn_MACInfo_Byte0_All"]),
      int(sync["Syn_MACInfo_Byte1_All"]),
      int(sync["Syn_MACInfo_Byte2_All"]),
    ])
    self.secoc_sync_seen = True

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # All signals — including the gateway-mirrored body/HMI messages — are read
    # from ADASBUS (Bus.pt). No IBUS1 tap required.
    pt_msgs = [
      # ESP / IMU / EPS / ADAS (native ADASBUS)
      ("ESP_0x115", 100),
      ("ESP_0x116", 100),
      ("ESP_0x318", 50),
      ("ESP_0x114", 50),
      ("ESP_0x120", 50),
      ("YRS_0x112", 100),
      ("YRS_0x113", 100),
      ("EPS_0x1C2", 50),
      ("EPS_0x1C4", 50),
      # ADAS module frames (0x313/0x314/0x31A/0x31C) live on the isolated intercept side
      # (bus 2), not on the bus we read — openpilot replaces that module. Cruise is taken
      # from the VCU's basic CC below instead.
      # Freqs are the real on-vehicle rates; over-declaring makes the CANParser flag a
      # message stale -> carState.canValid=False -> commIssue. Measured on ADASBUS.
      ("VCU_0x358", 10),    # basic cruise-control (CC) state + set speed (~10 Hz)
      ("ICC_0x531", 10),
      ("GW_Syn_All", 2),    # SecOC sync (~3 Hz)
      # Gateway-mirrored body/HMI (also present on ADASBUS)
      ("VCU_0x214", 50),    # gear, accel pedal %, brake
      ("BCM_0x343", 20),    # doors
      ("BCM_0x335", 20),    # blinkers
      ("ACU_0x159", 20),    # driver seatbelt
      ("MFS_0x514", 20),    # MFSS buttons (~20 Hz)
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_msgs, CANBUS.pt),
    }
