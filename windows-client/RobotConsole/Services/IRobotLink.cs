using System.Windows.Media.Imaging;
using RustRemoval.RobotConsole.Models;

namespace RustRemoval.RobotConsole.Services;

public interface IRobotLink
{
    event Action<TelemetrySnapshot>? OnTelemetryUpdated;
    event Action<BitmapSource>? OnVideoFrame;
    event Action<List<RustSpot>>? OnRustSpotsUpdated;
    event Action<LinkState>? OnLinkStateChanged;
    event Action<int, int>? OnLinkQualityUpdated;

    Task ConnectAsync();
    void Disconnect();
    void SetMode(OperationMode mode);
    void SetDriveSubmode(DriveSubmode submode);
    void SendDriveCommand(float x, float y, float rotate);
    void SendArmJogCommand(float dx, float dy, float dz, float dPitch, float dYaw);
    void SetArmHeight(float deltaMm);
    void ArmHome();
    void SetLaserPreset(string preset);
    void SetLaserParams(float powerPercent, float scanSpeedMmPerSec);
    void SendLaserCommand(bool enable);
    void SendEstop();
    void SendSnapshot();
    void SendRecordToggle(bool recording);
    void SetVideoQualityMode(VideoQualityMode mode);
}
