using System.Text.Json;
using RustRemoval.RobotConsole.Models;
using RustRemoval.RobotConsole.Services;

var host=args.Length>0 ? args[0] : "127.0.0.1";
using var http=new HttpClient {Timeout=TimeSpan.FromSeconds(3)};
async Task<JsonDocument> Report()=>JsonDocument.Parse(await http.GetStringAsync($"http://{host}:8080/diagnostics"));
using(var r=await Report()) {
    if(r.RootElement.GetProperty("arm").GetProperty("backend_mode").GetString()!="sim")
        throw new Exception("This test may only run against the simulated backend");
}
using var link=new RealRobotLink(new RobotLinkSettings {Mode="Real",RobotHost=host,UdpPort=9000,MjpegUrl=$"http://{host}:8080/stream.mjpg",VideoTransport="MJPEG",CommandIntervalMs=50,ControlTimeoutMs=350});
var compatible=new TaskCompletionSource();
var metrics=new TaskCompletionSource();
link.OnArmReport+=r=>{if(r.Compatible)compatible.TrySetResult();};
link.OnNetworkMetrics+=m=>{if(m.RttMs is not null && m.Completed>0)metrics.TrySetResult();};
using var cancel=new CancellationTokenSource();
var pulse=Task.Run(async()=>{while(!cancel.IsCancellationRequested){link.PulseUi();await Task.Delay(40);}});
await link.ConnectAsync();
await compatible.Task.WaitAsync(TimeSpan.FromSeconds(8));
await metrics.Task.WaitAsync(TimeSpan.FromSeconds(8));
await link.ExecuteArmActionAsync("recover");
await link.ExecuteArmActionAsync("enable");
await link.ExecuteArmActionAsync("set_home");
link.SetMode(OperationMode.Work);link.SendArmJogCommand(0,0,0,0,0);await Task.Delay(200);
double[] start;
using(var r=await Report())start=r.RootElement.GetProperty("arm").GetProperty("pose").EnumerateArray().Select(x=>x.GetDouble()).ToArray();
link.SendArmJogCommand(0,.04f,0,0,0);await Task.Delay(1200);
link.SendArmJogCommand(0,0,0,0,0);await Task.Delay(500);
using(var r=await Report()) {
    var root=r.RootElement;var arm=root.GetProperty("arm");
    if(root.GetProperty("fault_latched").GetBoolean() || arm.GetProperty("fault_latched").GetBoolean())throw new Exception(r.RootElement.ToString());
    var end=arm.GetProperty("pose").EnumerateArray().Select(x=>x.GetDouble()).ToArray();
    double distance=Math.Sqrt(Enumerable.Range(0,3).Sum(i=>Math.Pow(end[i]-start[i],2)));
    if(distance<.2)throw new Exception("No observed simulated movement");
    Console.WriteLine($"PASS: Windows RealRobotLink -> UDP -> bridge -> ROS Servo -> simulated feedback, displacement={distance:F3} mm");
}
await link.ExecuteArmActionAsync("home");await Task.Delay(2000);
using(var r=await Report()) {
    var arm=r.RootElement.GetProperty("arm");
    if(arm.GetProperty("fault_latched").GetBoolean())throw new Exception(arm.ToString());
    var home=arm.GetProperty("home_pose").EnumerateArray().Select(x=>x.GetDouble()).ToArray();
    var actual=arm.GetProperty("pose").EnumerateArray().Select(x=>x.GetDouble()).ToArray();
    var distance=Math.Sqrt(Enumerable.Range(0,3).Sum(i=>Math.Pow(actual[i]-home[i],2)));
    if(distance>.75)throw new Exception($"Simulated home did not reach recorded pose: {distance} mm");
    Console.WriteLine($"PASS: recorded pose return observed, residual={distance:F3} mm");
}
await link.ExecuteArmActionAsync("stop");
link.SendEstop();await Task.Delay(200);
using(var r=await Report())if(!r.RootElement.GetProperty("fault_latched").GetBoolean())throw new Exception("Manual stop failed");
await link.ExecuteArmActionAsync("recover");
link.Disconnect();await Task.Delay(400);
using(var r=await Report())if(!r.RootElement.GetProperty("fault_latched").GetBoolean())throw new Exception("Disconnect watchdog failed");
cancel.Cancel();await pulse;
Console.WriteLine("PASS: ACK metrics, version handshake, home actions, manual stop, explicit recovery and disconnect watchdog");
