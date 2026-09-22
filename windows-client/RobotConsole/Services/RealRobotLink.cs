using System.Diagnostics;
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;
using System.IO;
using System.Net;
using System.Net.Http;
using System.Net.Sockets;
using System.Windows.Media.Imaging;
using RustRemoval.RobotConsole.Models;

namespace RustRemoval.RobotConsole.Services;

/// <summary>
/// Ground v8 link using the existing 32-byte wire frame with a v8 speed handshake.
/// Events are marshalled back to the UI synchronization context captured at construction.
/// </summary>
public sealed class RealRobotLink : IRobotLink, IRobotLinkCapabilities, IDisposable
{
    private readonly RobotLinkSettings _settings;
    private readonly SynchronizationContext? _eventContext;
    private readonly object _gate = new();
    private readonly HttpClient _httpClient = new() { Timeout = Timeout.InfiniteTimeSpan };
    private UdpClient? _udp;
    private CancellationTokenSource? _cts;
    private Task[] _tasks = [];
    private RobotControlState _control = new(OperationMode.Standby, 0, 0, 0, 0, 0, 0, 0, 0, false, 55, 32, false);
    private readonly LinkMetrics _metrics = new();
    public event Action<NetworkMetrics>? OnNetworkMetrics;
    private ushort _sequence;
    private uint _sessionId;
    private long _clockOffset, _uiPulse, _lastAckAt, _lastClockSync;
    private ushort? _lastTelemetrySeq;
    private readonly SemaphoreSlim _sendGate=new(1,1);
    private bool _backup;
    private string _httpBase="";
    public event Action<string>? OnPathChanged;
    public event Action<SensorTelemetry>? OnSensorTelemetry;
    public string VideoTransport=>_settings.VideoTransport;
    public string ActiveWebRtcUrl=>_backup ? _settings.BackupWebRtcUrl! : _settings.WebRtcUrl;
    public double LinearSpeedLimit { get; private set; } = 50;
    public double AngularSpeedLimit { get; private set; } = 10;
    public double InputDeadband { get; private set; } = .02;
    public string LatestDiagnosticsJson { get; private set; } = "尚无诊断数据";
    public event Action? OnMotionCapabilities;
    public static bool TryReadMotionProfile(JsonElement root,out double linear,out double angular,out double deadband)
    {
        linear=angular=deadband=0;
        return root.TryGetProperty("bridge_version",out var version)&&version.GetString()=="8.1-piper-l-cpv"
            && root.TryGetProperty("control_profile",out var profile)&&profile.GetString()=="piper-l-cpv-v8.1"
            && root.TryGetProperty("motion_backend",out var backend)&&backend.GetString()=="ros2_moveit_servo"
            && root.TryGetProperty("real_executor_abi",out var abi)&&abi.GetString()=="piper_l:V189:rad_s:sdk_signs"
            && root.TryGetProperty("motion_capabilities",out var caps)
            && caps.TryGetProperty("linear_speed_mm_s",out var l)&&l.TryGetDouble(out linear)
            && caps.TryGetProperty("angular_speed_deg_s",out var a)&&a.TryGetDouble(out angular)
            && caps.TryGetProperty("input_deadband",out var d)&&d.TryGetDouble(out deadband)
            && double.IsFinite(linear)&&linear>=2&&linear<=50&&double.IsFinite(angular)&&angular>=1&&angular<=10
            && double.IsFinite(deadband)&&deadband>=0&&deadband<1;
    }
    public void PulseUi()=>Interlocked.Exchange(ref _uiPulse,Environment.TickCount64);
    public void VideoFramePresented()=>Interlocked.Exchange(ref _lastVideoAt,Environment.TickCount64);

    private long _startedAt;
    private long _lastTelemetryAt;
    private long _lastVideoAt;
    private bool _controlEstablished;
    private LinkState _reportedState = LinkState.Reconnecting;
    private bool _disposed;

    public RealRobotLink(RobotLinkSettings settings)
    {
        _settings = settings;
        _settings.Validate();
        _eventContext = SynchronizationContext.Current;
    }

    public event Action<TelemetrySnapshot>? OnTelemetryUpdated;
    public event Action<BitmapSource>? OnVideoFrame;
    public event Action<List<RustSpot>>? OnRustSpotsUpdated;
    public event Action<LinkState>? OnLinkStateChanged;
    public event Action<int, int>? OnLinkQualityUpdated { add { } remove { } }

    public event Action<string, double[]?>? OnArmDiagnostics;
    public event Action<bool>? OnArmReady;
    public event Action<ArmReport>? OnArmReport;
    private string? _actionToken;
    public async Task ExecuteArmActionAsync(string action)
    {
        var token = _actionToken ?? throw new InvalidOperationException("尚未连接 v8.1 桥接");
        // Cancel all motion and stop repeating the previous emergency bit first.
        Update(_ => new RobotControlState(OperationMode.Standby,0,0,0,0,0,0,0,0,false,0,0,false));
        await SendCurrentFrameAsync(CancellationToken.None);
        await Task.Delay(150);
        var udpPort = ((IPEndPoint?)_udp?.Client.LocalEndPoint)?.Port ?? throw new InvalidOperationException("控制链路未连接");
        var body = JsonSerializer.Serialize(new {action, token, udp_port=udpPort, request_id=Guid.NewGuid().ToString("N")});
        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(12));
        using var response = await _httpClient.PostAsync(new Uri(new Uri(_httpBase),"/arm/action"),
            new StringContent(body,Encoding.UTF8,"application/json"),timeout.Token);
        using var result = JsonDocument.Parse(await response.Content.ReadAsStringAsync(timeout.Token));
        if (!response.IsSuccessStatusCode || !result.RootElement.GetProperty("ok").GetBoolean())
            throw new InvalidOperationException(result.RootElement.TryGetProperty("error",out var error) ? error.GetString() : "操作失败");
    }

    public bool SupportsArmHeightStep => false;
    public bool SupportsArmHome => false;
    public bool SupportsArmOrientationStep => false;
    public bool SupportsSnapshot => false;
    public bool SupportsRecording => false;
    public bool SupportsRemoteEstopReset => false;

    public Task ConnectAsync()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        StopTransport(reportDisconnected: false);
        _metrics.Reset(); _sessionId=0;_lastTelemetrySeq=null;_lastAckAt=0;_actionToken=null;_lastClockSync=0;_controlEstablished=false;
        Update(_=>new RobotControlState(OperationMode.Standby,0,0,0,0,0,0,0,0,false,55,32,false));
        _httpBase=_backup ? _settings.BackupHttpUrl! : _settings.MjpegUrl;

        _startedAt = Environment.TickCount64;
        _lastTelemetryAt = default;
        _lastVideoAt = default;
        _reportedState = LinkState.Reconnecting;
        Raise(() => OnLinkStateChanged?.Invoke(LinkState.Reconnecting));
        Raise(() => OnRustSpotsUpdated?.Invoke([]));
        _cts = new CancellationTokenSource();
        var bind=_backup ? _settings.BackupLocalBindAddress : _settings.LocalBindAddress;
        _udp = new UdpClient(new IPEndPoint(string.IsNullOrWhiteSpace(bind)?IPAddress.Any:IPAddress.Parse(bind),0));
        _udp.Connect(_backup?_settings.BackupRobotHost!:_settings.RobotHost,_settings.UdpPort);
        try { _udp.Client.SetSocketOption(SocketOptionLevel.IP,SocketOptionName.TypeOfService,0xb8); } catch(SocketException) { }
        Raise(()=>OnPathChanged?.Invoke(_backup?"备用链路（恢复后需重新使能）":"主链路"));
        var token = _cts.Token;
        _tasks =
        [
            Task.Run(() => ControlLoopAsync(token), token),
            Task.Run(() => SessionLoopAsync(token),token),
            Task.Run(() => TelemetryLoopAsync(token), token),
            Task.Run(() => VideoLoopAsync(token), token),
            Task.Run(() => LinkMonitorLoopAsync(token), token),
            Task.Run(() => DiagnosticsLoopAsync(token), token),
        ];
        return Task.CompletedTask;
    }

    public void Disconnect() => StopTransport(reportDisconnected: true);
    public async Task SendNeutralBeforeDisconnectAsync()
    {
        Update(s=>s with {Mode=OperationMode.Standby,DriveX=0,DriveY=0,DriveRotate=0,ArmDx=0,ArmDy=0,ArmDz=0,ArmPitch=0,ArmYaw=0,LaserEnable=false});
        using var timeout=new CancellationTokenSource(500);
        try { await SendCurrentFrameAsync(timeout.Token); } catch(Exception ex) { Debug.WriteLine(ex); }
    }
    public async Task<string> GetRecentLogsAsync()
    {
        using var timeout=new CancellationTokenSource(2000);
        var json=await _httpClient.GetStringAsync(new Uri(new Uri(_httpBase),"/logs/recent"),timeout.Token);
        using var doc=JsonDocument.Parse(json);
        return JsonSerializer.Serialize(doc.RootElement,new JsonSerializerOptions {WriteIndented=true});
    }

    public void SetMode(OperationMode mode) => Update(state => state with { Mode = mode });
    public void SetDriveSubmode(DriveSubmode submode) { }
    public void SendDriveCommand(float x, float y, float rotate) =>
        Update(state => state with { DriveX = x, DriveY = y, DriveRotate = rotate });
    public void SendArmJogCommand(float dx, float dy, float dz, float dPitch, float dYaw) =>
        Update(state => state with { ArmDx = dx, ArmDy = dy, ArmDz = dz, ArmPitch = dPitch, ArmYaw = dYaw });

    // Protocol v1 has no discrete height/home/media command. Capability flags keep their UI disabled.
    public void SetArmHeight(float deltaMm) { }
    public void ArmHome() { }
    public void SendSnapshot() { }
    public void SendRecordToggle(bool recording) { }

    public void SetLaserPreset(string preset) { }
    public void SetLaserParams(float powerPercent, float scanSpeedMmPerSec) => Update(state => state with
    {
        LaserPower = (ushort)Math.Clamp(Math.Round(powerPercent), 0, 100),
        LaserSpeedTimes10 = (ushort)Math.Clamp(Math.Round(scanSpeedMmPerSec * 10), 0, ushort.MaxValue),
    });
    public void SendLaserCommand(bool enable) => Update(state => state with { LaserEnable = enable });

    public void SendEstop()
    {
        // Latch locally and send immediately in addition to the periodic stream.
        Update(state => state with
        {
            Mode = OperationMode.Standby,
            DriveX = 0, DriveY = 0, DriveRotate = 0,
            ArmDx = 0, ArmDy = 0, ArmDz = 0, ArmPitch = 0, ArmYaw = 0,
            LaserEnable = false,
            Estop = true,
        });
        _ = SendCurrentFrameAsync(CancellationToken.None);
    }

    public void SetVideoQualityMode(VideoQualityMode mode) { }

    private async Task DiagnosticsLoopAsync(CancellationToken token)
    {
        var url = new Uri(new Uri(_httpBase), "/diagnostics");
        while (!token.IsCancellationRequested)
        {
            try
            {
                using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                timeout.CancelAfter(500);
                long sent=Environment.TickCount64;
                var json = await _httpClient.GetStringAsync(url, timeout.Token).ConfigureAwait(false);
                long received=Environment.TickCount64;
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                var safeDetails=System.Text.Json.Nodes.JsonNode.Parse(json)!.AsObject();safeDetails.Remove("action_token");
                LatestDiagnosticsJson=safeDetails.ToJsonString(new JsonSerializerOptions { WriteIndented=true });
                var compatible=TryReadMotionProfile(root,out var linear,out var angular,out var deadband);
                if(compatible && (linear!=LinearSpeedLimit||angular!=AngularSpeedLimit||deadband!=InputDeadband)) {
                    LinearSpeedLimit=linear;AngularSpeedLimit=angular;InputDeadband=deadband;
                    Raise(()=>OnMotionCapabilities?.Invoke());
                }
                if(compatible && received-sent<=100 && root.TryGetProperty("monotonic_time",out var mt)) {
                    Interlocked.Exchange(ref _clockOffset,(long)(mt.GetDouble()*1000)-(sent+received)/2);
                    Interlocked.Exchange(ref _lastClockSync,received);
                }
                var arm = root.GetProperty("arm");
                var backendMode=arm.TryGetProperty("backend_mode",out var bm) ? bm.GetString() : "unknown";
                var backendText=backendMode switch {"sim"=>"ROS 模拟（不驱动实机）","readonly"=>"ROS 只读反馈（运动禁用）","real"=>"PIPER-L CPV 实机 · SDK 法兰",_=>"后端模式未知"};
                _actionToken = compatible && root.TryGetProperty("action_token",out var at) ? at.GetString() : null;
                var ready = compatible && _sessionId!=0 && arm.TryGetProperty("ready", out var ar) && ar.ValueKind == JsonValueKind.True
                    && root.TryGetProperty("fault_latched", out var fault) && fault.ValueKind == JsonValueKind.False
                    && !(root.TryGetProperty("input_rearm_required",out var rearm)&&rearm.GetBoolean());
                double[]? pose = null;
                if (arm.TryGetProperty("pose", out var p) && p.ValueKind == JsonValueKind.Array && p.GetArrayLength() == 6)
                    pose = p.EnumerateArray().Select(x => x.GetDouble()).ToArray();
                var error = root.GetProperty("last_arm_error").GetString();
                var currentError = root.TryGetProperty("blocking_reason",out var blocking) ? blocking.GetString() : arm.GetProperty("error").GetString();
                var enabled = "使能未知";
                if (arm.TryGetProperty("enabled", out var en) && en.ValueKind == JsonValueKind.Array && en.GetArrayLength() == 6)
                    enabled = en.EnumerateArray().All(x => x.GetBoolean()) ? "六轴已使能" : "未使能："+string.Join("、",en.EnumerateArray().Select((x,i)=>(x,i)).Where(v=>!v.x.GetBoolean()).Select(v=>$"J{v.i+1}"));
                var modeCode=arm.TryGetProperty("ctrl_mode",out var cm) && cm.ValueKind==JsonValueKind.Number ? cm.GetInt32() : -1;
                var modeText=modeCode switch {0=>"待机",1=>"CAN 控制",2=>"示教模式",_=>$"未知（{modeCode}）"};
                var faulted = root.TryGetProperty("fault_latched",out var fl) && fl.GetBoolean();
                if(faulted && root.TryGetProperty("fault_reason",out var reason)) currentError=reason.GetString();
                if(!compatible) currentError="版本或速度配置不匹配，请使用配套 v8.1 桥接";
                var message=$"运行方式：{backendText}\n当前异常：{(string.IsNullOrEmpty(currentError)?"无":currentError)}\n使能状态：{enabled}\n控制模式：{modeText}";
                if(pose is not null) message+=$"\nXYZ {pose[0]:F1}, {pose[1]:F1}, {pose[2]:F1} mm\nRPY {pose[3]:F1}, {pose[4]:F1}, {pose[5]:F1}°";
                else message+="\nXYZ / RPY：反馈过期或不可用";
                var report = new ArmReport(compatible,ready,faulted,
                    arm.TryGetProperty("status",out var sc) && sc.ValueKind==JsonValueKind.Number ? sc.GetInt32() : null,
                    enabled,pose,
                    arm.TryGetProperty("home_pose",out var hp) && hp.ValueKind==JsonValueKind.Array && (!arm.TryGetProperty("home_validated",out var hv) || hv.ValueKind==JsonValueKind.True),
                    root.TryGetProperty("returning_home",out var rh) && rh.GetBoolean(),
                    currentError ?? "");
                Raise(() => { OnArmDiagnostics?.Invoke(message, pose); OnArmReady?.Invoke(ready); OnArmReport?.Invoke(report); });
            }
            catch (Exception) when (!token.IsCancellationRequested)
            {
                _actionToken = null;
                Raise(() => { OnArmDiagnostics?.Invoke("当前异常：诊断连接中断\n使能状态：未知\n控制模式：未知\nXYZ / RPY：反馈不可用", null); OnArmReady?.Invoke(false);
                    OnArmReport?.Invoke(new(false,false,false,null,"使能未知",null,false,false,"诊断连接中断")); });
            }
            catch (OperationCanceledException) { return; }
            try { await Task.Delay(100, token).ConfigureAwait(false); }
            catch (OperationCanceledException) { return; }
        }
    }

    private async Task ControlLoopAsync(CancellationToken token)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(_settings.CommandIntervalMs));
        try
        {
            do { await SendCurrentFrameAsync(token).ConfigureAwait(false); }
            while (await timer.WaitForNextTickAsync(token).ConfigureAwait(false));
        }
        catch (OperationCanceledException) { }
        catch (Exception ex) { Debug.WriteLine($"UDP control loop stopped: {ex}"); }
    }

    private async Task SessionLoopAsync(CancellationToken token)
    {
        var requestId=Guid.NewGuid().ToString("N");
        while(!token.IsCancellationRequested && _sessionId==0)
        {
            try {
                var actionToken=_actionToken;
                if(actionToken!=null && _udp!=null) {
                    var port=((IPEndPoint)_udp.Client.LocalEndPoint!).Port;
                    var body=JsonSerializer.Serialize(new {action="session",token=actionToken,udp_port=port,request_id=requestId,control_profile="piper-l-cpv-v8.1"});
                    using var timeout=CancellationTokenSource.CreateLinkedTokenSource(token);timeout.CancelAfter(1000);
                    long sent=Environment.TickCount64;
                    using var response=await _httpClient.PostAsync(new Uri(new Uri(_httpBase),"/arm/action"),new StringContent(body,Encoding.UTF8,"application/json"),timeout.Token);
                    using var doc=JsonDocument.Parse(await response.Content.ReadAsStringAsync(timeout.Token));
                    long received=Environment.TickCount64;
                    if(response.IsSuccessStatusCode && doc.RootElement.GetProperty("ok").GetBoolean() && received-sent<=100 && doc.RootElement.TryGetProperty("control_profile",out var profile) && profile.GetString()=="piper-l-cpv-v8.1") {
                        var server=doc.RootElement.GetProperty("server_ms").GetUInt32();
                        Interlocked.Exchange(ref _clockOffset,(long)server-(sent+received)/2);
                        _lastClockSync=received;_sessionId=doc.RootElement.GetProperty("session").GetUInt32();
                    }
                }
            } catch(Exception) when(!token.IsCancellationRequested) { }
            try { await Task.Delay(500,token); } catch(OperationCanceledException) { return; }
        }
    }

    private async Task SendCurrentFrameAsync(CancellationToken token)
    {
        await _sendGate.WaitAsync(token);
        try {
            UdpClient? udp;RobotControlState state;ushort sequence;uint sid;
            var now=Environment.TickCount64;
            bool alive=now-Interlocked.Read(ref _uiPulse)<250;
            lock(_gate) {
                if(!alive || (_lastAckAt!=0 && now-_lastAckAt>350) || (_lastClockSync!=0 && now-_lastClockSync>5000))
                    _control=new(OperationMode.Standby,0,0,0,0,0,0,0,0,false,0,0,_control.Estop);
                udp=_udp;state=_control;sequence=_sequence++;sid=_sessionId;
            }
            if(udp==null || sid==0) return;
            uint stamp=unchecked((uint)(now+Interlocked.Read(ref _clockOffset)));
            var data=WireV6.Control(sid,sequence,stamp,alive,state);
            _metrics.Sent(sequence,stamp,now);
            await udp.SendAsync(data,token).ConfigureAwait(false);
        } catch(ObjectDisposedException) { }
        catch(SocketException ex) { Debug.WriteLine(ex); }
        finally { _sendGate.Release(); }
    }

    private async Task TelemetryLoopAsync(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                var udp = _udp;
                if (udp is null) return;
                var result = await udp.ReceiveAsync(token).ConfigureAwait(false);
                if(WireV6.Ack(result.Buffer,_sessionId,out var ackSeq,out var ackStamp)) {
                    if(_metrics.Acknowledge(ackSeq,ackStamp,Environment.TickCount64)) _lastAckAt=Environment.TickCount64;
                    continue;
                }
                if(!WireV6.Telemetry(result.Buffer,_sessionId,out var wire,out var stamp,out var sensors)) continue;
                if(_lastTelemetrySeq is ushort last && !WireV6.Newer(wire.Sequence,last)) continue;
                if(WireV6.Age(unchecked((uint)(Environment.TickCount64+Interlocked.Read(ref _clockOffset))),stamp) is < -100 or > 300) continue;
                _lastTelemetrySeq=wire.Sequence;
                _lastTelemetryAt=Environment.TickCount64;
                Raise(()=>OnSensorTelemetry?.Invoke(sensors!));
                var interlocks = Enumerable.Range(0, 5)
                    .Select(bit => (wire.InterlockBits & (1 << bit)) != 0).ToArray();
                var snapshot = new TelemetrySnapshot(
                    wire.BatteryPercent, wire.LatencyMs, wire.RollDeg, wire.PitchDeg,
                    wire.JointMargins.Select(value => (float)value).ToArray(),
                    interlocks, wire.LaserActive);
                Raise(() =>
                {
                    OnTelemetryUpdated?.Invoke(snapshot);
                    // Legacy telemetry latency is command age, not network RTT.
                    var metrics = _metrics.Snapshot(Environment.TickCount64);
                    OnNetworkMetrics?.Invoke(metrics);
                });
            }
        }
        catch (OperationCanceledException) { }
        catch (ObjectDisposedException) { }
        catch (Exception ex) { Debug.WriteLine($"UDP telemetry loop stopped: {ex}"); }
    }

    private async Task VideoLoopAsync(CancellationToken token)
    {
        if(_settings.VideoTransport=="WebRTC") return;
        while (!token.IsCancellationRequested)
        {
            try
            {
                using var response = await _httpClient.GetAsync(
                    _backup ? new Uri(new Uri(_settings.BackupHttpUrl!),new Uri(_settings.MjpegUrl).PathAndQuery).AbsoluteUri : _settings.MjpegUrl, HttpCompletionOption.ResponseHeadersRead, token).ConfigureAwait(false);
                response.EnsureSuccessStatusCode();
                await using var stream = await response.Content.ReadAsStreamAsync(token).ConfigureAwait(false);
                await ReadJpegsAsync(stream, token).ConfigureAwait(false);
            }
            catch (OperationCanceledException) { return; }
            catch (Exception ex)
            {
                Debug.WriteLine($"MJPEG reconnect: {ex.Message}");
                try { await Task.Delay(750, token).ConfigureAwait(false); }
                catch (OperationCanceledException) { return; }
            }
        }
    }

    private async Task ReadJpegsAsync(Stream stream, CancellationToken token)
    {
        var readBuffer = new byte[8192];
        using var jpeg = new MemoryStream();
        var inside = false;
        var previous = -1;
        while (!token.IsCancellationRequested)
        {
            var count = await stream.ReadAsync(readBuffer, token).ConfigureAwait(false);
            if (count == 0) throw new EndOfStreamException("MJPEG stream ended");
            for (var i = 0; i < count; i++)
            {
                var current = readBuffer[i];
                if (!inside)
                {
                    if (previous == 0xFF && current == 0xD8)
                    {
                        inside = true;
                        jpeg.SetLength(0);
                        jpeg.WriteByte(0xFF);
                        jpeg.WriteByte(0xD8);
                    }
                }
                else
                {
                    jpeg.WriteByte(current);
                    if (jpeg.Length > 8 * 1024 * 1024)
                        throw new InvalidDataException("MJPEG frame exceeded 8 MiB");
                    if (previous == 0xFF && current == 0xD9)
                    {
                        PublishJpeg(jpeg.ToArray());
                        inside = false;
                    }
                }
                previous = current;
            }
        }
    }

    private void PublishJpeg(byte[] bytes)
    {
        using var stream = new MemoryStream(bytes, writable: false);
        var image = new BitmapImage();
        image.BeginInit();
        image.CacheOption = BitmapCacheOption.OnLoad;
        image.StreamSource = stream;
        image.EndInit();
        image.Freeze();
        _lastVideoAt = Environment.TickCount64;
        Raise(() => OnVideoFrame?.Invoke(image));
    }

    private async Task LinkMonitorLoopAsync(CancellationToken token)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(50));
        try
        {
            while (await timer.WaitForNextTickAsync(token).ConfigureAwait(false))
            {
                var now = Environment.TickCount64;
                LinkState next;
                if(_lastTelemetryAt!=0 && _lastAckAt!=0) _controlEstablished=true;
                // No motion is permitted while the initial HTTP/UDP handshake is
                // incomplete. Its 500 ms retry must not trip the 350 ms RUN watchdog.
                if (!_controlEstablished)
                    next = (now - _startedAt) < 3000
                        ? LinkState.Reconnecting : LinkState.ControlLost;
                else if ((now - _lastTelemetryAt) > _settings.ControlTimeoutMs || now-_lastAckAt>350)
                    next = LinkState.ControlLost;
                else if (_lastVideoAt == default || (now - _lastVideoAt) > _settings.VideoTimeoutMs)
                    next = LinkState.VideoLost;
                else
                    next = LinkState.Connected;
                ReportState(next);
                if(next==LinkState.ControlLost && now-Math.Max(_lastTelemetryAt,_startedAt)>1500) {
                    if(!string.IsNullOrWhiteSpace(_settings.BackupRobotHost)) _backup=!_backup;
                    await ConnectAsync();return;
                }
                var metrics = _metrics.Snapshot(Environment.TickCount64);
                Raise(() => OnNetworkMetrics?.Invoke(metrics));
            }
        }
        catch (OperationCanceledException) { }
    }

    private void ReportState(LinkState state)
    {
        if (state == _reportedState) return;
        _reportedState = state;
        if (state == LinkState.ControlLost)
        {
            // Never replay stale motion when a link returns. Only zeroed standby heartbeats seek reconnection.
            Update(current => current with
            {
                Mode = OperationMode.Standby,
                DriveX = 0, DriveY = 0, DriveRotate = 0,
                ArmDx = 0, ArmDy = 0, ArmDz = 0, ArmPitch = 0, ArmYaw = 0,
                LaserEnable = false, Estop = current.Estop || _controlEstablished,
            });
        }
        Raise(() => OnLinkStateChanged?.Invoke(state));
    }

    private void Update(Func<RobotControlState, RobotControlState> update)
    {
        lock (_gate) _control = update(_control);
    }

    private void Raise(Action callback)
    {
        if (_eventContext is null) callback();
        else _eventContext.Post(_ => callback(), null);
    }

    private void StopTransport(bool reportDisconnected)
    {
        var cts = _cts;
        _cts = null;
        cts?.Cancel();
        lock (_gate)
        {
            _udp?.Dispose();
            _udp = null;
        }
        cts?.Dispose();
        _tasks = [];
        if (reportDisconnected) ReportState(LinkState.ControlLost);
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        StopTransport(reportDisconnected: false);
        _httpClient.Dispose();
    }
}
