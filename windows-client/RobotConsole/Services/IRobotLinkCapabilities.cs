namespace RustRemoval.RobotConsole.Services;

/// <summary>
/// Optional feature discovery for commands absent from protocol v1.
/// Unsupported operations must be disabled instead of pretending they were sent.
/// </summary>
public interface IRobotLinkCapabilities
{
    bool SupportsArmHeightStep { get; }
    bool SupportsArmHome { get; }
    bool SupportsArmOrientationStep { get; }
    bool SupportsSnapshot { get; }
    bool SupportsRecording { get; }
    bool SupportsRemoteEstopReset { get; }
}
