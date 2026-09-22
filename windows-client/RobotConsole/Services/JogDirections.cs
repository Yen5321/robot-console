using System.IO;
using System.Text.Json;
namespace RustRemoval.RobotConsole.Services;

public sealed class JogDirections
{
    public int RightYSign { get; set; } = 1;
    public int UpZSign { get; set; } = 1;
    public int ExtendXSign { get; set; } = 1;
    public bool Calibrated { get; set; }
    public string? ConfirmedAtUtc { get; set; }
    public void Validate()
    {
        if (Math.Abs(RightYSign) != 1 || Math.Abs(UpZSign) != 1 || Math.Abs(ExtendXSign) != 1)
            throw new InvalidDataException("点动方向必须为 +1 或 -1");
    }
    public static JogDirections Load(string path)
    {
        var result = File.Exists(path) ? JsonSerializer.Deserialize<JogDirections>(File.ReadAllText(path)) ?? throw new InvalidDataException("方向配置为空") : new JogDirections();
        result.Validate();
        return result;
    }
    public void Save(string path)
    {
        Validate();
        File.WriteAllText(path + ".tmp", JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true }));
        File.Move(path + ".tmp", path, true);
    }
    public float[] MapJoystick(double horizontal, double vertical)
    {
        if (!double.IsFinite(horizontal) || !double.IsFinite(vertical)) return new float[5];
        var length = Math.Max(1, Math.Sqrt(horizontal*horizontal + vertical*vertical));
        return [0, (float)(RightYSign*horizontal/length), (float)(UpZSign*vertical/length), 0, 0];
    }
    public float[] MapAxis(string axis) => axis switch
    {
        "x+" => [ExtendXSign, 0, 0, 0, 0], "x-" => [-ExtendXSign, 0, 0, 0, 0],
        "pitch+" => [0, 0, 0, 1, 0], "pitch-" => [0, 0, 0, -1, 0],
        "yaw+" => [0, 0, 0, 0, 1], "yaw-" => [0, 0, 0, 0, -1],
        _ => throw new ArgumentException("未知点动轴")
    };
}
