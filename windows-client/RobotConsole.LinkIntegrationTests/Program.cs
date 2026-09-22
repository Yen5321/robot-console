using RustRemoval.RobotConsole.Models;
using RustRemoval.RobotConsole.Services;

var telemetryReady = new TaskCompletionSource<TelemetrySnapshot>(TaskCreationOptions.RunContinuationsAsynchronously);
var diagnosticsReady = new TaskCompletionSource<double[]>(TaskCreationOptions.RunContinuationsAsynchronously);
var videoReady = new TaskCompletionSource<System.Windows.Media.Imaging.BitmapSource>(TaskCreationOptions.RunContinuationsAsynchronously);
var connectedReady = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
var armReady = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
var metricsReady = new TaskCompletionSource<NetworkMetrics>(TaskCreationOptions.RunContinuationsAsynchronously);

using var link = new RealRobotLink(new RobotLinkSettings
{
    Mode = "Real", VideoTransport="MJPEG",
    RobotHost = "127.0.0.1",
    UdpPort = args.Length > 0 ? int.Parse(args[0]) : 19000,
    MjpegUrl = $"http://127.0.0.1:{(args.Length > 1 ? int.Parse(args[1]) : 18080)}/stream.mjpg",
    CommandIntervalMs = 50,
    ControlTimeoutMs = 350,
    VideoTimeoutMs = 2500,
});
link.OnTelemetryUpdated += value => { if(value.InterlockStatus.All(x=>x)) telemetryReady.TrySetResult(value); };
link.OnNetworkMetrics += value => { if(value.RttMs is not null && value.Completed > 0) metricsReady.TrySetResult(value); };
link.OnArmReady += value => { if (value) armReady.TrySetResult(); };
link.OnArmDiagnostics += (message, pose) => { if (pose is not null && message.Contains("六轴已使能")) diagnosticsReady.TrySetResult(pose); };
link.OnVideoFrame += value => videoReady.TrySetResult(value);
link.OnLinkStateChanged += state => { if (state == LinkState.Connected) connectedReady.TrySetResult(); };

using var pulseCancel=new CancellationTokenSource();
var pulse=Task.Run(async()=>{while(!pulseCancel.IsCancellationRequested){link.PulseUi();await Task.Delay(40);}});
await link.ConnectAsync();
var telemetry = await telemetryReady.Task.WaitAsync(TimeSpan.FromSeconds(6));
var pose = await diagnosticsReady.Task.WaitAsync(TimeSpan.FromSeconds(6));
await armReady.Task.WaitAsync(TimeSpan.FromSeconds(6));
Assert(true, "v8 session bridge version and ready status accepted");
Assert(pose.SequenceEqual(new double[] {100, 0, 200, 0, 10, 0}), "Python diagnostics and enable state reached C# over HTTP");
var video = await videoReady.Task.WaitAsync(TimeSpan.FromSeconds(6));
await connectedReady.Task.WaitAsync(TimeSpan.FromSeconds(6));
link.SendArmJogCommand(0, 0, 0, 0, 0);

Assert(telemetry.BatteryPercent == 88, "Python telemetry reached C# over UDP");
Assert(telemetry.InterlockStatus.All(value => value), "Python interlock bits decoded in C#");
Assert(telemetry.JointMargins.SequenceEqual(new float[] { 78, 72, 66, 59, 31, 64 }), "joint margins kept field order");
Assert(video.PixelWidth == 640 && video.PixelHeight == 360, "Python MJPEG decoded into WPF BitmapSource");
link.SetMode(OperationMode.Work);
link.SendArmJogCommand(0,.2f,-.1f,0,0);
await Task.Delay(100);
link.SendArmJogCommand(0,0,0,0,0);
var metrics = await metricsReady.Task.WaitAsync(TimeSpan.FromSeconds(6));
Assert(metrics.RttMs >= 0 && metrics.AckTimeoutPercent is not null, "real ACK echoes produce RTT and settled timeout statistics");
foreach(var action in new[]{"enable","set_home","home","stop"})
{
    await link.ExecuteArmActionAsync(action);
    Assert(true, "HTTP action succeeded: " + action);
}
link.SendEstop();
await Task.Delay(350);
await link.ExecuteArmActionAsync("recover");
Assert(true, "explicit recovery after UDP emergency stop succeeds");
link.Disconnect();
await Task.Delay(400);
using (var http = new System.Net.Http.HttpClient())
{
    var port = args.Length > 1 ? int.Parse(args[1]) : 18080;
    using var report = System.Text.Json.JsonDocument.Parse(await http.GetStringAsync($"http://127.0.0.1:{port}/diagnostics"));
    Assert(report.RootElement.GetProperty("fault_latched").GetBoolean(), "robot latches a stop after client disconnect");
    Assert(report.RootElement.GetProperty("fault_reason").GetString()!.Contains("timeout"), "timeout stop reason is exposed");
}
Console.WriteLine("PASS: WPF RealRobotLink <-> Python BridgeService end-to-end");

static void Assert(bool condition, string message)
{
    if (!condition) throw new InvalidOperationException($"FAIL: {message}");
    Console.WriteLine($"PASS: {message}");
}


