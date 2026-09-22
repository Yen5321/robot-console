namespace RustRemoval.RobotConsole.Models;

public enum OperationMode { Standby, Drive, Work }
public enum DriveSubmode { Standard, Crab, Spin }
public enum VideoQualityMode { SmoothPriority, ClarityPriority }
public enum LinkState { Connected, VideoLost, ControlLost, Reconnecting }

public record TelemetrySnapshot(
    int BatteryPercent,
    int LatencyMs,
    float RollDeg,
    float PitchDeg,
    float[] JointMargins,
    bool[] InterlockStatus,
    bool LaserActive);

public record RustSpot(int Id, float BoxX, float BoxY, float BoxW, float BoxH, float Confidence);
