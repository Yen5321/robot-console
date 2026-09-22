using RustRemoval.RobotConsole.Models;

namespace RustRemoval.RobotConsole.Services;

public interface IMockRobotLinkDebug
{
    void ForceLinkState(LinkState state);
    void SetInterlock(int index, bool value);
}
