namespace RustRemoval.RobotConsole.Services;
public record ArmReport(bool Compatible, bool Ready, bool Faulted, int? Status,
    string Enabled, double[]? Pose, bool HasHome, bool ReturningHome, string Error);
