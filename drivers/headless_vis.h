// No-op stand-ins for VSG/Irrlicht visual systems so demos can run headless (campaign tool).
#pragma once
#include "chrono/assets/ChVisualSystem.h"
namespace chrono {
struct HeadlessVisStub {
#define HV_NOOP(name) template <class... A> void name(A&&...) {}
    HV_NOOP(AttachSystem) HV_NOOP(SetWindowTitle) HV_NOOP(SetCameraVertical) HV_NOOP(AddCamera)
    HV_NOOP(SetWindowSize) HV_NOOP(SetBackgroundColor) HV_NOOP(EnableSkyTexture) HV_NOOP(SetCameraAngleDeg)
    HV_NOOP(SetLightIntensity) HV_NOOP(SetLightDirection) HV_NOOP(EnableShadows) HV_NOOP(Initialize)
    HV_NOOP(Render) HV_NOOP(BeginScene) HV_NOOP(EndScene) HV_NOOP(AddLogo) HV_NOOP(AddSkyBox)
    HV_NOOP(AddTypicalLights) HV_NOOP(AddLight) HV_NOOP(SetSymbolScale) HV_NOOP(EnableContactDrawing) HV_NOOP(BindAll)
    HV_NOOP(AddUserEventReceiver) HV_NOOP(ShowInfoPanel) HV_NOOP(AddGrid)
#undef HV_NOOP
    bool Run() { return true; }
};
namespace vsg3d { using ChVisualSystemVSG = HeadlessVisStub; }
namespace irrlicht { using ChVisualSystemIrrlicht = HeadlessVisStub; enum class ContactsDrawMode { CONTACT_NORMALS, CONTACT_DISTANCES, CONTACT_FORCES_N, CONTACT_FORCES }; }
}  // namespace chrono
