using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using RustRemoval.RobotConsole.Models;

namespace RustRemoval.RobotConsole.Services;

/// <summary>
/// Deterministic, UI-thread mock. This is the only class in the current project that
/// simulates an external transport; View and ViewModel depend solely on IRobotLink.
/// </summary>
public sealed class MockRobotLink : IRobotLink, IMockRobotLinkDebug, IRobotLinkCapabilities, IDisposable
{
    private readonly DispatcherTimer _telemetryTimer = new() { Interval = TimeSpan.FromMilliseconds(100) };
    private readonly DispatcherTimer _videoTimer = new() { Interval = TimeSpan.FromMilliseconds(250) };
    private readonly DispatcherTimer _spotTimer = new() { Interval = TimeSpan.FromSeconds(2) };
    private readonly Random _random = new(4066);
    private readonly bool[] _interlocks = [true, true, true, true, true];
    private DateTime _connectedAt;
    private int _battery = 76;
    private int _tick;
    private bool _laserActive;
    private LinkState _linkState = LinkState.Reconnecting;

    public event Action<TelemetrySnapshot>? OnTelemetryUpdated;
    public event Action<BitmapSource>? OnVideoFrame;
    public event Action<List<RustSpot>>? OnRustSpotsUpdated;
    public event Action<LinkState>? OnLinkStateChanged;
    public event Action<int, int>? OnLinkQualityUpdated;

    public bool SupportsArmHeightStep => true;
    public bool SupportsArmHome => true;
    public bool SupportsArmOrientationStep => true;
    public bool SupportsSnapshot => true;
    public bool SupportsRecording => true;
    public bool SupportsRemoteEstopReset => true;

    public MockRobotLink()
    {
        _telemetryTimer.Tick += (_, _) => PublishTelemetry();
        _videoTimer.Tick += (_, _) => PublishVideoFrame();
        _spotTimer.Tick += (_, _) => PublishRustSpots();
    }

    public async Task ConnectAsync()
    {
        ForceLinkState(LinkState.Reconnecting);
        await Task.Delay(650);
        _connectedAt = DateTime.Now;
        _telemetryTimer.Start();
        _videoTimer.Start();
        _spotTimer.Start();
        ForceLinkState(LinkState.Connected);
        PublishTelemetry();
        PublishVideoFrame();
        PublishRustSpots();
    }

    public void Disconnect()
    {
        _telemetryTimer.Stop();
        _videoTimer.Stop();
        _spotTimer.Stop();
        ForceLinkState(LinkState.ControlLost);
    }

    public void SetMode(OperationMode mode) { }
    public void SetDriveSubmode(DriveSubmode submode) { }
    public void SendDriveCommand(float x, float y, float rotate) { }
    public void SendArmJogCommand(float dx, float dy, float dz, float dPitch, float dYaw) { }
    public void SetArmHeight(float deltaMm) { }
    public void ArmHome() { }
    public void SetLaserPreset(string preset) { }
    public void SetLaserParams(float powerPercent, float scanSpeedMmPerSec) { }
    public void SendLaserCommand(bool enable) => _laserActive = enable;
    public void SendEstop() => _laserActive = false;
    public void SendSnapshot() { }
    public void SendRecordToggle(bool recording) { }
    public void SetVideoQualityMode(VideoQualityMode mode) { }

    public void ForceLinkState(LinkState state)
    {
        _linkState = state;
        if (state is LinkState.VideoLost or LinkState.ControlLost)
            _laserActive = false;
        OnLinkStateChanged?.Invoke(state);
    }

    public void SetInterlock(int index, bool value)
    {
        if (index < 0 || index >= _interlocks.Length) return;
        _interlocks[index] = value;
        if (!value) _laserActive = false;
        PublishTelemetry();
    }

    private void PublishTelemetry()
    {
        _tick++;
        if (_tick % 600 == 0 && _battery > 0) _battery--;

        // A repeatable short spike every ~30 seconds demonstrates warning thresholds.
        var elapsed = (DateTime.Now - _connectedAt).TotalSeconds;
        var spike = elapsed > 0 && elapsed % 30 < 2.2;
        var latency = spike ? _random.Next(310, 475) : _random.Next(50, 151);
        var lossPermille = spike ? _random.Next(35, 100) : _random.Next(0, 8);
        var margins = new[]
        {
            73f + Jitter(3), 66f + Jitter(3), 58f + Jitter(3),
            49f + Jitter(3), 20f + Jitter(4), 62f + Jitter(3)
        };
        OnTelemetryUpdated?.Invoke(new TelemetrySnapshot(
            _battery, latency, Jitter(3), Jitter(3), margins, [.. _interlocks], _laserActive));
        OnLinkQualityUpdated?.Invoke(latency, lossPermille);
    }

    private float Jitter(float radius) => (float)((_random.NextDouble() * 2 - 1) * radius);

    private void PublishRustSpots() => OnRustSpotsUpdated?.Invoke(
    [
        new RustSpot(1, .12f, .18f, .24f, .20f, .94f),
        new RustSpot(2, .56f, .28f, .19f, .25f, .87f),
        new RustSpot(3, .39f, .66f, .17f, .15f, .79f)
    ]);

    private void PublishVideoFrame()
    {
        if (_linkState is LinkState.VideoLost or LinkState.ControlLost) return;
        const int width = 960, height = 540;
        var visual = new DrawingVisual();
        using (var dc = visual.RenderOpen())
        {
            var background = new LinearGradientBrush(Color.FromRgb(28, 49, 58), Color.FromRgb(8, 18, 25), 90);
            dc.DrawRectangle(background, null, new Rect(0, 0, width, height));
            dc.DrawRectangle(new SolidColorBrush(Color.FromRgb(67, 80, 83)), null, new Rect(80, 110, 800, 330));
            dc.DrawLine(new Pen(new SolidColorBrush(Color.FromRgb(133, 91, 54)), 30), new Point(110, 390), new Point(850, 155));
            dc.DrawLine(new Pen(new SolidColorBrush(Color.FromRgb(183, 111, 48)), 12), new Point(150, 175), new Point(815, 415));
            dc.DrawEllipse(new SolidColorBrush(Color.FromRgb(136, 66, 31)), null, new Point(278, 220), 82, 52);
            dc.DrawEllipse(new SolidColorBrush(Color.FromRgb(158, 79, 35)), null, new Point(625, 280), 68, 73);
            var gridPen = new Pen(new SolidColorBrush(Color.FromArgb(38, 255, 255, 255)), 1);
            for (var x = 0; x < width; x += 80) dc.DrawLine(gridPen, new Point(x, 0), new Point(x, height));
            for (var y = 0; y < height; y += 60) dc.DrawLine(gridPen, new Point(0, y), new Point(width, y));
            var text = new FormattedText($"D435  MOCK  {DateTime.Now:HH:mm:ss.fff}", System.Globalization.CultureInfo.CurrentCulture,
                FlowDirection.LeftToRight, new Typeface("Consolas"), 21, Brushes.White, 1.0);
            dc.DrawText(text, new Point(20, 18));
            dc.DrawLine(new Pen(Brushes.LimeGreen, 2), new Point(455, 270), new Point(505, 270));
            dc.DrawLine(new Pen(Brushes.LimeGreen, 2), new Point(480, 245), new Point(480, 295));
        }
        var bitmap = new RenderTargetBitmap(width, height, 96, 96, PixelFormats.Pbgra32);
        bitmap.Render(visual);
        bitmap.Freeze();
        OnVideoFrame?.Invoke(bitmap);
    }

    public void Dispose()
    {
        _telemetryTimer.Stop();
        _videoTimer.Stop();
        _spotTimer.Stop();
    }
}
