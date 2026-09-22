using System.Windows.Media.Imaging;
using RustRemoval.RobotConsole.Models;
using RustRemoval.RobotConsole.Services;
using RustRemoval.RobotConsole.ViewModels;

var knownControl = RobotWireProtocol.PackControl(
    0x1234, 0x78563412, 1,
    new RobotControlState(
        OperationMode.Work, -1, 0, 1,
        .001f, -.002f, .003f, -.004f, .005f,
        true, 100, 32, false));
Assert(knownControl.Length == 30, "C# control frame is exactly 30 bytes");
Assert(Convert.ToHexString(knownControl).Equals(
    "3412123456780218FC0000E8030100FEFF0300FCFF050001640020000001",
    StringComparison.OrdinalIgnoreCase), "C# control bytes match Python known vector");

var telemetryBytes = Convert.FromHexString(
    "FFFF5817000000A03F000020C00102030405061F01");
Assert(RobotWireProtocol.TryParseTelemetry(telemetryBytes, out var decoded), "C# accepts Python telemetry frame");
Assert(decoded.Sequence == 65535 && decoded.BatteryPercent == 88 && decoded.LatencyMs == 23 &&
       Math.Abs(decoded.RollDeg - 1.25f) < .001 && Math.Abs(decoded.PitchDeg + 2.5f) < .001 &&
       decoded.JointMargins.SequenceEqual(new byte[] { 1, 2, 3, 4, 5, 6 }) &&
       decoded.InterlockBits == 0x1F && decoded.LaserActive,
    "C# telemetry fields match Python layout");

var link = new TestRobotLink();
var vm = new MainViewModel(link);
link.SetState(LinkState.Connected);
link.SetTelemetry([true, true, true, true, true]);

Assert(vm.IsStandbyMode, "initial mode is standby");
Assert(!vm.CanStartLaser, "laser cannot start in standby");

vm.SwitchToWorkCommand.Execute(null);
Assert(vm.IsWorkMode, "work mode switch");
Assert(vm.CanJogArm, "healthy arm needs no calibration checkbox");
Assert(vm.JogSpeedMmS==2, "default jog speed is 2 mm/s");
vm.JogSpeedMmS=50;
vm.JogAngularSpeedDegS=10;
vm.CommissioningMode = true;
Assert(vm.BeginArmInput("axis"), "explicit commissioning permits low speed tests");
vm.UpdateArmAxis("x+");
Assert(link.LastArm.SequenceEqual(new float[] { 1, 0, 0, 0, 0 }), "extend routes to X only");
Assert(!vm.BeginArmInput("joystick"), "second pointer cannot replace active input");
vm.UpdateArmAxis("pitch-");
Assert(link.LastArm.SequenceEqual(new float[] { 0, 0, 0, -1, 0 }), "pitch hold routes to RY jog");
vm.UpdateArmAxis("yaw+");
Assert(link.LastArm.SequenceEqual(new float[] { 0, 0, 0, 0, 1 }), "yaw hold routes to RZ jog");
vm.StopArmJog();
Assert(link.LastArm.All(x => x == 0), "release clears all jog axes");
vm.UpdateArmAxis("x+");
Assert(link.LastArm.All(x => x == 0), "stale move event after release cannot restart input");
Assert(vm.BeginArmInput("joystick"), "new pointer can start joystick");
vm.UpdateArmJoystick(1, 1);
Assert(link.LastArm[0] == 0 && Math.Abs(link.LastArm[1] - 0.707) < .0001 && Math.Abs(link.LastArm[2] - 0.707) < .0001 && link.LastArm[3] == 0 && link.LastArm[4] == 0, "diagonal joystick maps Y/Z and clamps vector length");
vm.ReverseRight = true;
Assert(link.LastArm.All(x => x == 0) && !vm.XChecked, "direction change cancels input and resets confirmation");
vm.BeginArmInput("joystick"); vm.UpdateArmJoystick(1, 0);
Assert(link.LastArm.SequenceEqual(new float[] {0,-1,0,0,0}), "right sign inversion preserves native axes");
vm.StopArmJog();
vm.XChecked = vm.YChecked = vm.ZChecked = true;
vm.SaveDirectionsCommand.Execute(null);
Assert(vm.CalibrationText.Contains("已现场确认") && !vm.CommissioningMode, "calibration requires three confirmations and exits test mode");
vm.BeginArmInput("joystick"); vm.UpdateArmJoystick(0, 1);
vm.SwitchToStandbyCommand.Execute(null);
vm.SwitchToWorkCommand.Execute(null);
vm.UpdateArmJoystick(0, 1);
Assert(link.LastArm.All(x => x == 0), "mode switch needs a new press before movement");
var savedPath = Path.Combine(Path.GetTempPath(), Guid.NewGuid() + ".json");
try {
    new JogDirections { RightYSign=-1, UpZSign=1, ExtendXSign=-1, Calibrated=true }.Save(savedPath);
    var saved = JogDirections.Load(savedPath);
    Assert(saved.Calibrated && saved.RightYSign == -1 && saved.ExtendXSign == -1, "direction configuration roundtrips");
} finally { File.Delete(savedPath); }
Assert(!vm.CanStartLaser, "unintegrated laser stays disabled even when interlocks are satisfied");
vm.ToggleLaserCommand.Execute(null);
Assert(link.LastLaserCommand != true && !vm.IsLaserActive, "placeholder cannot receive an enable request");

link.SetTelemetry([true, true, false, true, true]);
Assert(!vm.CanStartLaser && !vm.IsLaserActive, "failed robot interlock blocks laser");
Assert(vm.LaserActionHint.Contains("未接入"), "hardware integration status is explicit");

link.SetTelemetry([true, true, true, true, true]);
link.SetState(LinkState.VideoLost);
Assert(!vm.CanStartLaser && !vm.Interlocks[5].IsSatisfied, "video status still updates independently of laser availability");

link.SetState(LinkState.Connected);
vm.ToggleEmergencyCommand.Execute(null);
Assert(vm.IsEmergencyStopped && link.EstopCount == 1, "emergency stop locks the UI and calls transport");
Assert(!vm.SwitchToDriveCommand.CanExecute(null), "mode changes blocked after emergency stop");
vm.UpdateArmAxis("x+");
Assert(link.LastArm.All(x => x == 0), "emergency stop blocks axis jog");
vm.ToggleEmergencyCommand.Execute(null);
Assert(!vm.IsEmergencyStopped, "explicit reset restores controls only while connected");

link.SetState(LinkState.ControlLost);
Assert(!vm.IsControlSurfaceEnabled, "control loss disables normal control surface");
Assert(!vm.SwitchToDriveCommand.CanExecute(null), "control loss prevents regular mode commands");
Assert(vm.ReconnectCommand.CanExecute(null), "control loss leaves reconnect available");
vm.UpdateArmAxis("x+");
Assert(link.LastArm.All(x => x == 0), "control loss blocks axis jog");

var metric = new LinkMetrics();
Assert(metric.Snapshot(0).RttMs is null && metric.Snapshot(0).AckTimeoutPercent is null, "network metrics start unknown");
metric.Sent(1,1,0); metric.Sent(2,2,10); metric.Sent(3,3,20);
metric.Acknowledge(1,1,35); metric.Acknowledge(3,3,50);
metric.Acknowledge(3,3,80); // Duplicate cannot rewrite RTT.
Assert(metric.Snapshot(100).RttMs==30, "RTT uses matching echoed packet and ignores duplicate ACK");
Assert(metric.Snapshot(100).AckTimeoutPercent is null, "unsettled window does not fabricate zero loss");
var measured=metric.Snapshot(1020);
Assert(measured.Completed==3 && measured.TimedOut==1 && Math.Abs(measured.AckTimeoutPercent!.Value-100.0/3)<.001, "ACK timeout ratio includes missing confirmation");
metric.Acknowledge(2,2,1100);
Assert(metric.Snapshot(1100).TimedOut==1, "late ACK does not erase deadline miss");
Assert(metric.Snapshot(7000).RttMs is null && metric.Snapshot(7000).AckTimeoutPercent is null, "expired measurement is unknown");
Console.WriteLine("PASS: protocol, axis jog and safety assertions");
Assert(WireV6.Crc("123456789"u8)==0x29b1,"CRC16 standard check vector");
var v6=WireV6.Control(0x11223344,0x1234,0x78563412,true,new RobotControlState(OperationMode.Work,-1,0,1,.001f,-.002f,.003f,-.004f,.005f,true,100,32,false));
Assert(Convert.ToHexString(v6).Equals("443322113412123456781A18FC0000E8030100FEFF0300FCFF05006420008FC0",StringComparison.OrdinalIgnoreCase),"C# and Python exact v6 32-byte control vector");
for(int bit=0;bit<256;bit++) {var damaged=(byte[])v6.Clone();damaged[bit/8]^=(byte)(1<<(bit%8));if(WireV6.Valid(damaged,32))throw new Exception("Undetected single bit corruption");}
Assert(true,"all 256 single bit flips rejected");
Assert(WireV6.Newer(0,65535)&&!WireV6.Newer(65535,0)&&!WireV6.Newer(8,8),"sequence wrap without accepting duplicates/reordering");
Assert(WireV6.Age(10,0xfffffff0)==26,"monotonic timestamp wrap");

// Profile gate is also used by the live diagnostics loop before granting a session.
using(var profile=System.Text.Json.JsonDocument.Parse("""{"bridge_version":"8.1-piper-l-cpv","control_profile":"piper-l-cpv-v8.1","motion_backend":"ros2_moveit_servo","real_executor_abi":"piper_l:V189:rad_s:sdk_signs","motion_capabilities":{"linear_speed_mm_s":50,"angular_speed_deg_s":10,"input_deadband":0.02}}""")) {
    Assert(RealRobotLink.TryReadMotionProfile(profile.RootElement,out var linear,out var angular,out _),"v8 matching profile accepted");
    Assert(linear==50&&angular==10,"negotiated speed maxima parsed independently");
    foreach(var old in new[]{profile.RootElement.GetRawText().Replace("8.1-piper-l-cpv","6.0-industrial"),profile.RootElement.GetRawText().Replace("piper-l-cpv-v8.1","legacy"),profile.RootElement.GetRawText().Replace(":50",":500")}) {
        using var mismatch=System.Text.Json.JsonDocument.Parse(old);
        Assert(!RealRobotLink.TryReadMotionProfile(mismatch.RootElement,out _,out _,out _),"old or incompatible bridge cannot grant motion");
    }
}
var settingsPath=System.IO.Path.Combine(System.IO.Path.GetTempPath(),"robotlink-v8-test-"+Guid.NewGuid()+".json");
try {
    var settings=new RobotLinkSettings { Mode="Real",RobotHost="10.1.2.3",UdpPort=19000,MjpegUrl="http://10.1.2.3:18080/stream.mjpg",LocalBindAddress="10.1.2.4" };
    settings.Save(settingsPath);
    settings.RobotHost="10.1.2.5";settings.Save(settingsPath);
    Assert(RobotLinkSettings.Load(settingsPath).RobotHost=="10.1.2.5"&&RobotLinkSettings.Load(settingsPath+".bak").RobotHost=="10.1.2.3","connection save roundtrip and backup");
    settings.LocalBindAddress="not-an-IP";
    bool rejected=false;try{settings.Save(settingsPath);}catch(InvalidOperationException){rejected=true;}
    Assert(rejected&&RobotLinkSettings.Load(settingsPath).LocalBindAddress=="10.1.2.4","invalid connection does not overwrite previous settings");
} finally {System.IO.File.Delete(settingsPath);System.IO.File.Delete(settingsPath+".bak");}

// Startup has no established control link; a slow handshake is not a RUN timeout.
using(var startupLink=new RealRobotLink(new RobotLinkSettings {Mode="Real",RobotHost="127.0.0.1",MjpegUrl="http://127.0.0.1:18711/stream.mjpg"})) {
    var flags=System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance;
    var report=typeof(RealRobotLink).GetMethod("ReportState",flags)!;
    var state=typeof(RealRobotLink).GetField("_control",flags)!;
    report.Invoke(startupLink,[LinkState.ControlLost]);
    Assert(!((RobotControlState)state.GetValue(startupLink)!).Estop,"initial handshake timeout does not synthesize remote emergency stop");
    report.Invoke(startupLink,[LinkState.Connected]);
    typeof(RealRobotLink).GetField("_controlEstablished",flags)!.SetValue(startupLink,true);
    startupLink.SendArmJogCommand(0,1,0,0,0);
    report.Invoke(startupLink,[LinkState.ControlLost]);
    var stopped=(RobotControlState)state.GetValue(startupLink)!;
    Assert(stopped.Estop&&stopped.ArmDy==0&&stopped.Mode==OperationMode.Standby,"established control loss still clears motion and requests stop");
}
using(var manualLink=new RealRobotLink(new RobotLinkSettings {Mode="Real",RobotHost="127.0.0.1",MjpegUrl="http://127.0.0.1:18711/stream.mjpg"})) {
    manualLink.SendEstop();
    var flags=System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance;
    typeof(RealRobotLink).GetMethod("ReportState",flags)!.Invoke(manualLink,[LinkState.ControlLost]);
    Assert(((RobotControlState)typeof(RealRobotLink).GetField("_control",flags)!.GetValue(manualLink)!).Estop,"manual stop remains latched during startup");
}

static void Assert(bool condition, string message)
{
    if (!condition) throw new InvalidOperationException($"FAIL: {message}");
    Console.WriteLine($"PASS: {message}");
}

#pragma warning disable CS0067
sealed class TestRobotLink : IRobotLink
{
    public event Action<TelemetrySnapshot>? OnTelemetryUpdated;
    public event Action<BitmapSource>? OnVideoFrame;
    public event Action<List<RustSpot>>? OnRustSpotsUpdated;
    public event Action<LinkState>? OnLinkStateChanged;
    public event Action<int, int>? OnLinkQualityUpdated;
    public bool? LastLaserCommand { get; private set; }
    public int EstopCount { get; private set; }

    public Task ConnectAsync() { SetState(LinkState.Connected); return Task.CompletedTask; }
    public void Disconnect() { }
    public void SetMode(OperationMode mode) { }
    public void SetDriveSubmode(DriveSubmode submode) { }
    public void SendDriveCommand(float x, float y, float rotate) { }
    public float[] LastArm { get; private set; } = new float[5];
    public void SendArmJogCommand(float dx, float dy, float dz, float dPitch, float dYaw) => LastArm = [dx, dy, dz, dPitch, dYaw];
    public void SetArmHeight(float deltaMm) { }
    public void ArmHome() { }
    public void SetLaserPreset(string preset) { }
    public void SetLaserParams(float powerPercent, float scanSpeedMmPerSec) { }
    public void SendLaserCommand(bool enable) => LastLaserCommand = enable;
    public void SendEstop() { EstopCount++; LastLaserCommand = false; }
    public void SendSnapshot() { }
    public void SendRecordToggle(bool recording) { }
    public void SetVideoQualityMode(VideoQualityMode mode) { }
    public void SetState(LinkState state) => OnLinkStateChanged?.Invoke(state);
    public void SetTelemetry(bool[] interlocks) => OnTelemetryUpdated?.Invoke(
        new TelemetrySnapshot(76, 80, 0, 0, [75, 70, 65, 60, 20, 55], interlocks, LastLaserCommand == true));
}
#pragma warning restore CS0067

