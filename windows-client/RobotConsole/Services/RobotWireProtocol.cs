using System.Buffers.Binary;
using RustRemoval.RobotConsole.Models;

namespace RustRemoval.RobotConsole.Services;

public readonly record struct RobotControlState(
    OperationMode Mode,
    float DriveX, float DriveY, float DriveRotate,
    float ArmDx, float ArmDy, float ArmDz, float ArmPitch, float ArmYaw,
    bool LaserEnable, ushort LaserPower, ushort LaserSpeedTimes10,
    bool Estop);

public readonly record struct RobotWireTelemetry(
    ushort Sequence, byte BatteryPercent, ushort LatencyMs,
    float RollDeg, float PitchDeg, byte[] JointMargins,
    byte InterlockBits, bool LaserActive);

/// <summary>Protocol v1 encoder/decoder shared by the real transport and contract tests.</summary>
public static class RobotWireProtocol
{
    public const int ControlFrameSize = 30;
    public const int TelemetryFrameSize = 21;

    public static byte[] PackControl(ushort sequence, uint timestamp, byte heartbeat, RobotControlState state)
    {
        var data = new byte[ControlFrameSize];
        var span = data.AsSpan();
        BinaryPrimitives.WriteUInt16LittleEndian(span[0..2], sequence);
        BinaryPrimitives.WriteUInt32LittleEndian(span[2..6], timestamp);
        span[6] = (byte)state.Mode;
        WriteNormalized(span[7..9], state.DriveX);
        WriteNormalized(span[9..11], state.DriveY);
        WriteNormalized(span[11..13], state.DriveRotate);
        WriteNormalized(span[13..15], state.ArmDx);
        WriteNormalized(span[15..17], state.ArmDy);
        WriteNormalized(span[17..19], state.ArmDz);
        WriteNormalized(span[19..21], state.ArmPitch);
        WriteNormalized(span[21..23], state.ArmYaw);
        span[23] = state.LaserEnable ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteUInt16LittleEndian(span[24..26], (ushort)Math.Min(100, (int)state.LaserPower));
        BinaryPrimitives.WriteUInt16LittleEndian(span[26..28], state.LaserSpeedTimes10);
        span[28] = state.Estop ? (byte)1 : (byte)0;
        span[29] = (byte)(heartbeat & 1);
        return data;
    }

    public static bool TryParseTelemetry(ReadOnlySpan<byte> data, out RobotWireTelemetry telemetry)
    {
        telemetry = default;
        if (data.Length != TelemetryFrameSize) return false;
        var battery = data[2];
        var margins = data[13..19].ToArray();
        if (battery > 100 || margins.Any(value => value > 100) || data[20] > 1) return false;
        telemetry = new RobotWireTelemetry(
            BinaryPrimitives.ReadUInt16LittleEndian(data[0..2]),
            battery,
            BinaryPrimitives.ReadUInt16LittleEndian(data[3..5]),
            ReadSingle(data[5..9]),
            ReadSingle(data[9..13]),
            margins,
            data[19],
            data[20] == 1);
        return true;
    }

    private static void WriteNormalized(Span<byte> destination, float value)
    {
        var wire = (short)Math.Round(Math.Clamp(value, -1f, 1f) * 1000f);
        BinaryPrimitives.WriteInt16LittleEndian(destination, wire);
    }

    private static float ReadSingle(ReadOnlySpan<byte> source) =>
        BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source));
}
