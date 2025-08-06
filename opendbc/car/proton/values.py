from dataclasses import dataclass, field
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts

# Todo
HUD_MULTIPLIER = 1.035
Ecu = structs.CarParams.Ecu
GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

class CarControllerParams:
  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    # make sure Proton only has one max steer torque value
    assert(len(CP.lateralParams.torqueV) == 1)

    # for torque limit calculation
    self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh

@dataclass
class ProtonCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

@dataclass
class ProtonPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "proton_general_pt"})

class CAR(Platforms):
  PROTON_S70 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton S70")],
    CarSpecs(mass=1300., wheelbase=2.627, steerRatio=15.0),
  )
  PROTON_X50 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X50")],
    CarSpecs(mass=1370., wheelbase=2.6, steerRatio=15.0)
  )
  PROTON_X90 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X90")],
    CarSpecs(mass=1705., wheelbase=2.805, steerRatio=15.0)
  )

class CanBus:
  main_bus = 0
  cam_bus = 1

DBC = CAR.create_dbc_map()
