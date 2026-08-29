import struct
import socket
import time

import usb1

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.usb import CHESTNUT_USB_IDS

CHESTNUT_STATE_SOCKET = "\0chestnutState"


class ChestnutUsb:
  def __init__(self):
    self.handle = None

  def close(self) -> None:
    if self.handle is not None:
      try:
        self.handle.close()
      except Exception:
        pass
      self.handle = None

  def read(self) -> tuple[int, int, bool, int]:
    if self.handle is None:
      context = usb1.USBContext()
      for vendor_id, product_id in CHESTNUT_USB_IDS:
        if (handle := context.openByVendorIDAndProductID(vendor_id, product_id, skip_on_error=True)) is not None:
          self.handle = handle
          break
      if self.handle is None:
        context.close()
        raise usb1.USBErrorNoDevice
    try:
      voltage, current, fault = struct.unpack('<Hh?', bytes(self.handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100)))
    except Exception:
      self.close()
      raise
    try:
      pcie = self.handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100)[0]
    except Exception:
      self.close()
      pcie = 0
    return voltage, current, fault, pcie


def _chestnut_telemetry_thread(end_event) -> None:
  pm = messaging.PubMaster(["chestnutState"])
  sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
  sock.bind(CHESTNUT_STATE_SOCKET)
  sock.setblocking(False)
  usb = ChestnutUsb()
  dt = 1. / SERVICE_LIST["chestnutState"].frequency
  gpu_state = None
  gpu_state_time = 0.
  while not end_event.wait(dt):
    try:
      try:
        while data := sock.recv(4096):
          gpu_state = messaging.log_from_bytes(data)
          gpu_state_time = time.monotonic()
      except BlockingIOError:
        pass
      msg = messaging.new_message("chestnutState")
      if gpu_state is not None and gpu_state.valid and time.monotonic() - gpu_state_time < 1.:
        msg.chestnutState = gpu_state.chestnutState
      state = msg.chestnutState
      state.supplyVoltage, state.supplyCurrent, state.supplyFault, state.pcieLtssm = usb.read()
      msg.valid = True
      pm.send("chestnutState", msg)
    except Exception:
      usb.close()
  usb.close()


def chestnut_telemetry_thread(end_event) -> None:
  while not end_event.is_set():
    try:
      _chestnut_telemetry_thread(end_event)
    except Exception:
      end_event.wait(1.)
