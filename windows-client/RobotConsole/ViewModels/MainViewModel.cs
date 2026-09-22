using System.Collections.ObjectModel;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using RustRemoval.RobotConsole.Models;
using RustRemoval.RobotConsole.Services;

namespace RustRemoval.RobotConsole.ViewModels;

public sealed class MainViewModel : ObservableObject, IDisposable
{
    private readonly IRobotLink _robotLink;
    private readonly DispatcherTimer _uiHeartbeat=new() { Interval=TimeSpan.FromMilliseconds(50) };
    private uint _sensorMask;
    public string BatteryDisplay=>_robotLink is RealRobotLink && (_sensorMask&32)==0 ? "--" : $"{BatteryPercent}%";
    public string BaseAttitudeDisplay=>_robotLink is RealRobotLink && (_sensorMask&16)==0 ? "姿态传感器未接入" : $"横滚 {RollDeg:F1}° 俯仰 {PitchDeg:F1}°";
    private string _sensorText="传感器：等待有效遥测",_pathText="主链路";
    public string SensorText {get=>_sensorText;private set=>Set(ref _sensorText,value);}
    public string PathText {get=>_pathText;private set=>Set(ref _pathText,value);}
    public bool UsesWebRtc=>_robotLink is RealRobotLink r && r.VideoTransport=="WebRTC";
    public string WebRtcUrl=>_robotLink is RealRobotLink r ? r.ActiveWebRtcUrl : "";
    public void VideoPresented() { if(_robotLink is RealRobotLink r) r.VideoFramePresented(); }

    private ArmReport? _armReport;
    private bool _actionBusy;
    private string _networkText="-- / --", _actionNotice="", _homeStatus="尚未记录零位";
    public string NetworkText { get=>_networkText; private set=>Set(ref _networkText,value); }
    public string ActionNotice { get=>_actionNotice; private set=>Set(ref _actionNotice,value); }
    public string HomeStatus { get=>_homeStatus; private set=>Set(ref _homeStatus,value); }
    public string ArmBlockHint => _actionBusy ? "操作执行中，请等待确认" :
        LinkState is LinkState.ControlLost or LinkState.Reconnecting ? "控制链路未就绪" :
        _robotLink is RealRobotLink && _armReport?.Compatible!=true ? "等待配套 v8.1 桥接与速度握手" :
        IsEmergencyStopped ? "停止已锁存：需显式恢复后重新按下" :
        !_backendReady ? "点动已禁用："+(_armReport?.Error??"等待新鲜反馈") :
        ArmReturning ? "回零中：点动已禁用，可点击停止回零" :
        !IsWorkMode ? "点动需先切换作业模式" : "";
    public RelayCommand EnableArmCommand {get;}
    public RelayCommand RecoverArmCommand {get;}
    public RelayCommand SetHomeCommand {get;}
    public RelayCommand ReturnHomeCommand {get;}
    public RelayCommand StopHomeCommand {get;}
    public RelayCommand AlwaysEstopCommand {get;}
    private bool CanArmAction => !_actionBusy && _armReport?.Compatible==true && LinkState is LinkState.Connected or LinkState.VideoLost;
    private bool ArmReturning => _armReport?.ReturningHome==true;
    private async Task ArmAction(string action)
    {
        if (!CanArmAction || _robotLink is not RealRobotLink real) return;
        StopArmJog(); _robotLink.SendDriveCommand(0,0,0); _robotLink.SendLaserCommand(false);
        _driveX=_driveY=0; _rotation=0; IsLaserActive=false;
        Mode=OperationMode.Standby;
        _actionBusy=true; RaiseSafetyState(); ActionNotice="正在执行…";
        try {
            await real.ExecuteArmActionAsync(action);
            ActionNotice=action switch {"enable"=>"CAN 控制与六轴使能已确认；切换作业模式点动", "recover"=>"恢复已确认；切换作业模式，重新按下才运动", "set_home"=>"当前实际位姿已记录", "home"=>"正在返回记录位姿", _=>"已下发当前位置保持"};
        } catch(Exception ex) { ActionNotice="操作未完成："+ex.Message; }
        finally { _actionBusy=false; RaiseSafetyState(); }
    }

    private readonly IMockRobotLinkDebug? _debugLink;
    private readonly IRobotLinkCapabilities? _capabilities;
    private readonly DispatcherTimer _feedbackTimer = new() { Interval = TimeSpan.FromSeconds(1.6) };
    private OperationMode _mode = OperationMode.Standby;
    private DriveSubmode _driveSubmode = DriveSubmode.Standard;
    private LinkState _linkState = LinkState.Reconnecting;
    private BitmapSource? _videoFrame;
    private RustSpotViewModel? _selectedSpot;
    private int _batteryPercent = 76;
    private int _latencyMs;
    private int _packetLossPermille;
    private float _rollDeg;
    private float _pitchDeg;
    private bool _isEmergencyStopped;
    private bool _isLaserActive;
    private bool _isRecording;
    private string _feedbackText = string.Empty;
    private float _driveX;
    private float _driveY;
    private double _rotation;
    private bool _slowSpeed;
    private double _frontLeftAngle;
    private double _frontRightAngle;
    private double _rearLeftAngle;
    private double _rearRightAngle;
    private double _armHeight = 150;
    private double _armPitch;
    private double _armYaw;
    private float _armX;
    private float _armY;
    private double _laserPower = 55;
    private double _scanSpeed = 3.2;
    private int _debugInterlockIndex;

    private readonly JogDirections _directions;
    private readonly string? _directionsPath;
    private string? _activeArmInput;
    private bool _commissioning;
    private bool _backendReady;
    private bool _xChecked, _yChecked, _zChecked;
    public event Action? ArmInputCancelled;
    public MainViewModel(IRobotLink robotLink, JogDirections? directions = null, string? directionsPath = null)
    {
        _robotLink = robotLink;
        EnableArmCommand=new RelayCommand(async()=>await ArmAction("enable"),()=>CanArmAction && !ArmReturning);
        RecoverArmCommand=new RelayCommand(async()=>await ArmAction("recover"),()=>CanArmAction && (_armReport?.Faulted==true || _armReport?.Status==1));
        SetHomeCommand=new RelayCommand(async()=>await ArmAction("set_home"),()=>CanArmAction && _backendReady && !ArmReturning);
        ReturnHomeCommand=new RelayCommand(async()=>await ArmAction("home"),()=>CanArmAction && _backendReady && _armReport?.HasHome==true && !ArmReturning);
        StopHomeCommand=new RelayCommand(async()=>await ArmAction("stop"),()=>CanArmAction && ArmReturning);
        AlwaysEstopCommand=new RelayCommand(()=> { StopArmJog(); _robotLink.SendEstop(); IsEmergencyStopped=true; IsLaserActive=false; });

        _directions = directions ?? new JogDirections();
        _directions.Validate();
        _directionsPath = directionsPath;
        _backendReady = robotLink is not RealRobotLink;
        SaveDirectionsCommand = new RelayCommand(SaveDirections, () => XChecked && YChecked && ZChecked && !_directions.Calibrated);
        _debugLink = robotLink as IMockRobotLinkDebug;
        _capabilities = robotLink as IRobotLinkCapabilities;

        for (var i = 1; i <= 6; i++) JointMargins.Add(new JointMarginViewModel($"J{i}"));
        string[] names = ["车辆稳定停止", "机械臂姿态安全域", "末端距离合规", "防护挡板到位", "作业区域无人员", "视频信号正常"];
        foreach (var name in names) Interlocks.Add(new InterlockItemViewModel(name, robotLink is not RealRobotLink && name != "视频信号正常"));

        SwitchToStandbyCommand = new RelayCommand(() => SwitchMode(OperationMode.Standby), CanChangeMode);
        SwitchToDriveCommand = new RelayCommand(() => SwitchMode(OperationMode.Drive), CanChangeMode);
        SwitchToWorkCommand = new RelayCommand(() => SwitchMode(OperationMode.Work), CanChangeMode);
        SetStandardDriveCommand = new RelayCommand(() => SetDriveSubmode(DriveSubmode.Standard), CanSendNormalControl);
        SetCrabDriveCommand = new RelayCommand(() => SetDriveSubmode(DriveSubmode.Crab), CanSendNormalControl);
        SetSpinDriveCommand = new RelayCommand(() => SetDriveSubmode(DriveSubmode.Spin), CanSendNormalControl);
        ToggleSpeedCommand = new RelayCommand(() => SlowSpeed = !SlowSpeed, CanSendNormalControl);
        HeightUpCommand = new RelayCommand(() => AdjustHeight(10), () => CanUseArm() && SupportsArmHeightStep);
        HeightDownCommand = new RelayCommand(() => AdjustHeight(-10), () => CanUseArm() && SupportsArmHeightStep);
        PitchUpCommand = new RelayCommand(() => AdjustPitch(5), () => CanUseArm() && SupportsArmOrientationStep);
        PitchDownCommand = new RelayCommand(() => AdjustPitch(-5), () => CanUseArm() && SupportsArmOrientationStep);
        YawUpCommand = new RelayCommand(() => AdjustYaw(5), () => CanUseArm() && SupportsArmOrientationStep);
        YawDownCommand = new RelayCommand(() => AdjustYaw(-5), () => CanUseArm() && SupportsArmOrientationStep);
        ArmHomeCommand = new RelayCommand(ArmHome, () => CanUseArm() && SupportsArmHome);
        LightPresetCommand = new RelayCommand(() => ApplyPreset("light", 30, 5.0), CanUseWorkControls);
        MediumPresetCommand = new RelayCommand(() => ApplyPreset("medium", 55, 3.2), CanUseWorkControls);
        HeavyPresetCommand = new RelayCommand(() => ApplyPreset("heavy", 80, 1.8), CanUseWorkControls);
        ToggleLaserCommand = new RelayCommand(ToggleLaser, () => IsLaserActive || CanStartLaser);
        ToggleEmergencyCommand = new RelayCommand(ToggleEmergency, () => !IsEmergencyStopped || CanResetEmergency);
        SnapshotCommand = new RelayCommand(Snapshot, () => CanSendNormalControl() && SupportsSnapshot);
        ToggleRecordCommand = new RelayCommand(ToggleRecord, () => CanSendNormalControl() && SupportsRecording);
        ReconnectCommand = new RelayCommand(async () => await ReconnectAsync(), () => LinkState is LinkState.ControlLost or LinkState.Reconnecting);
        SelectSpotCommand = new RelayCommand<RustSpotViewModel>(SelectSpot);
        DebugSetConnectedCommand = new RelayCommand(() => _debugLink?.ForceLinkState(LinkState.Connected));
        DebugSetVideoLostCommand = new RelayCommand(() => _debugLink?.ForceLinkState(LinkState.VideoLost));
        DebugSetControlLostCommand = new RelayCommand(() => _debugLink?.ForceLinkState(LinkState.ControlLost));
        DebugSetReconnectingCommand = new RelayCommand(() => _debugLink?.ForceLinkState(LinkState.Reconnecting));
        DebugToggleInterlockCommand = new RelayCommand(ToggleDebugInterlock);
        WaitForVideoCommand = new RelayCommand(() => FeedbackText = "等待视频链路自动恢复…");

        _feedbackTimer.Tick += (_, _) => { _feedbackTimer.Stop(); FeedbackText = string.Empty; };
        if (_robotLink is RealRobotLink real)
        {
            _uiHeartbeat.Tick+=(_,_)=>real.PulseUi();real.PulseUi();_uiHeartbeat.Start();
            real.OnPathChanged+=path=> { PathText=path;Raise(nameof(WebRtcUrl));StopArmJog();Mode=OperationMode.Standby; };
            real.OnSensorTelemetry+=sensor=>{ _sensorMask=sensor.ValidMask;Raise(nameof(BatteryDisplay));Raise(nameof(BaseAttitudeDisplay)); SensorText=$"电池电压：{((sensor.ValidMask&4)!=0 ? sensor.BatteryVoltageV.ToString("F1")+" V" : "未接入")}\n磁吸力：{((sensor.ValidMask&8)!=0 ? sensor.MagneticForceN.ToString("F1")+" N" : "未接入")}\n电机电流：{((sensor.ValidMask&2)!=0 ? string.Join(" / ",sensor.MotorCurrentA.Select(x=>x.ToString("F2")))+" A" : "未接入")}"; };
            real.OnNetworkMetrics += m => NetworkText=$"{(m.RttMs is double t ? $"{t:F0} ms" : "--")} / {(m.AckTimeoutPercent is double p ? $"{p:F1}%" : "采样中")}";
            real.OnArmReport += ApplyArmReport;
            real.OnMotionCapabilities += () => {
                StopArmJog();
                JogSpeedMmS=Math.Min(JogSpeedMmS,real.LinearSpeedLimit);
                JogAngularSpeedDegS=Math.Min(JogAngularSpeedDegS,real.AngularSpeedLimit);
                Raise(nameof(JogSpeedMaximum));Raise(nameof(JogAngularMaximum));
            };
            real.OnArmDiagnostics += (message, pose) =>
            {
                ArmDiagnostics = message;
                if (pose is not null) { ArmHeight = pose[2]; ArmPitch = pose[4]; ArmYaw = pose[5]; }
                HasArmPose = pose is not null;
            };
            real.OnArmReady += ready =>
            {
                if (_backendReady && !ready) StopArmJog();
                _backendReady = ready;
                Raise(nameof(CanJogArm));
            };
        }
        _robotLink.OnTelemetryUpdated += HandleTelemetry;
        _robotLink.OnVideoFrame += frame => VideoFrame = frame;
        _robotLink.OnRustSpotsUpdated += HandleRustSpots;
        _robotLink.OnLinkStateChanged += HandleLinkState;
        _robotLink.OnLinkQualityUpdated += (latency, loss) => { LatencyMs = latency; PacketLossPermille = loss; };
    }

    public void ApplyArmReport(ArmReport report)
    {
        _armReport=report;
        _backendReady=report.Ready;
        if(!report.Ready || report.ReturningHome) StopArmJog();
        IsEmergencyStopped=report.Faulted || report.Status==1;
        HomeStatus=report.ReturningHome ? "正在回零" : report.HasHome ? "零位已记录" : "尚未记录零位";
        RaiseSafetyState();
    }

    public ObservableCollection<JointMarginViewModel> JointMargins { get; } = [];
    public ObservableCollection<InterlockItemViewModel> Interlocks { get; } = [];
    public IEnumerable<InterlockItemViewModel> VisibleInterlocks => Interlocks.Where(x => x.Name is not ("防护挡板到位" or "作业区域无人员"));
    public string DiagnosticDetails => _robotLink is RealRobotLink real ? real.LatestDiagnosticsJson : "模拟模式：无实机诊断";
    public async Task<string> RecentDiagnosticLogAsync() => _robotLink is RealRobotLink real ? await real.GetRecentLogsAsync() : "模拟模式";
    public async Task DisconnectForSettingsAsync()
    {
        StopArmJog();_robotLink.SendDriveCommand(0,0,0);_robotLink.SendLaserCommand(false);
        Mode=OperationMode.Standby;_robotLink.SetMode(Mode);
        if(_robotLink is RealRobotLink real) await real.SendNeutralBeforeDisconnectAsync();
        _robotLink.Disconnect();
    }
    public ObservableCollection<RustSpotViewModel> RustSpots { get; } = [];

    public bool SupportsArmHeightStep => _capabilities?.SupportsArmHeightStep ?? true;
    public bool SupportsArmHome => _capabilities?.SupportsArmHome ?? true;
    public bool SupportsArmOrientationStep => _capabilities?.SupportsArmOrientationStep ?? true;
    public bool SupportsSnapshot => _capabilities?.SupportsSnapshot ?? true;
    public bool SupportsRecording => _capabilities?.SupportsRecording ?? true;
    public bool SupportsRemoteEstopReset => _capabilities?.SupportsRemoteEstopReset ?? true;
    public bool HasProtocolV1Limitations => !SupportsArmHeightStep || !SupportsArmHome || !SupportsArmOrientationStep || !SupportsSnapshot || !SupportsRecording;
    public bool CanResetEmergency => SupportsRemoteEstopReset && LinkState == LinkState.Connected;

    public RelayCommand SwitchToStandbyCommand { get; }
    public RelayCommand SwitchToDriveCommand { get; }
    public RelayCommand SwitchToWorkCommand { get; }
    public RelayCommand SetStandardDriveCommand { get; }
    public RelayCommand SetCrabDriveCommand { get; }
    public RelayCommand SetSpinDriveCommand { get; }
    public RelayCommand ToggleSpeedCommand { get; }
    public RelayCommand HeightUpCommand { get; }
    public RelayCommand HeightDownCommand { get; }
    public RelayCommand PitchUpCommand { get; }
    public RelayCommand PitchDownCommand { get; }
    public RelayCommand YawUpCommand { get; }
    public RelayCommand YawDownCommand { get; }
    public RelayCommand ArmHomeCommand { get; }
    public RelayCommand LightPresetCommand { get; }
    public RelayCommand MediumPresetCommand { get; }
    public RelayCommand HeavyPresetCommand { get; }
    public RelayCommand ToggleLaserCommand { get; }
    public RelayCommand ToggleEmergencyCommand { get; }
    public RelayCommand SnapshotCommand { get; }
    public RelayCommand ToggleRecordCommand { get; }
    public RelayCommand ReconnectCommand { get; }
    public RelayCommand<RustSpotViewModel> SelectSpotCommand { get; }
    public RelayCommand DebugSetConnectedCommand { get; }
    public RelayCommand DebugSetVideoLostCommand { get; }
    public RelayCommand DebugSetControlLostCommand { get; }
    public RelayCommand DebugSetReconnectingCommand { get; }
    public RelayCommand DebugToggleInterlockCommand { get; }
    public RelayCommand WaitForVideoCommand { get; }

    public OperationMode Mode { get => _mode; private set { if (Set(ref _mode, value)) RaiseModeState(); } }
    public bool IsStandbyMode => Mode == OperationMode.Standby;
    public bool IsDriveMode => Mode == OperationMode.Drive;
    public bool IsWorkMode => Mode == OperationMode.Work;
    public string ModeText => Mode switch { OperationMode.Drive => "行驶模式", OperationMode.Work => "作业模式", _ => "待机" };

    public DriveSubmode DriveSubmode { get => _driveSubmode; private set { if (Set(ref _driveSubmode, value)) RaiseDriveSubmodeState(); } }
    public bool IsStandardDrive => DriveSubmode == DriveSubmode.Standard;
    public bool IsCrabDrive => DriveSubmode == DriveSubmode.Crab;
    public bool IsSpinDrive => DriveSubmode == DriveSubmode.Spin;

    public LinkState LinkState { get => _linkState; private set { if (Set(ref _linkState, value)) RaiseSafetyState(); } }
    public string LinkStateText => LinkState switch { LinkState.Connected => "控制与视频正常", LinkState.VideoLost => "图像丢失", LinkState.ControlLost => "控制链路断开", _ => "正在重连" };
    public bool IsVideoLost => LinkState == LinkState.VideoLost;
    public bool IsControlLost => LinkState == LinkState.ControlLost;
    public bool IsReconnecting => LinkState == LinkState.Reconnecting;
    public bool IsLinkConnected => LinkState == LinkState.Connected;

    public int BatteryPercent { get => _batteryPercent; private set => Set(ref _batteryPercent, value); }
    public int LatencyMs { get => _latencyMs; private set { if (Set(ref _latencyMs, value)) { Raise(nameof(IsLatencyWarning)); Raise(nameof(IsLatencyCritical)); } } }
    public bool IsLatencyWarning => LatencyMs >= 200 && LatencyMs <= 400;
    public bool IsLatencyCritical => LatencyMs > 400;
    public int PacketLossPermille { get => _packetLossPermille; private set => Set(ref _packetLossPermille, value); }
    public float RollDeg { get => _rollDeg; private set => Set(ref _rollDeg, value); }
    public float PitchDeg { get => _pitchDeg; private set => Set(ref _pitchDeg, value); }
    public string MinimumJointText
    {
        get
        {
            var item = JointMargins.OrderBy(x => x.Value).FirstOrDefault();
            return item is null ? "--" : $"{item.Name}  {item.Value:F0}%";
        }
    }
    public bool IsMinimumJointWarning => JointMargins.Count > 0 && JointMargins.Min(x => x.Value) < 25;

    public BitmapSource? VideoFrame { get => _videoFrame; private set => Set(ref _videoFrame, value); }
    public RustSpotViewModel? SelectedSpot { get => _selectedSpot; private set => Set(ref _selectedSpot, value); }
    public bool IsEmergencyStopped { get => _isEmergencyStopped; private set { if (Set(ref _isEmergencyStopped, value)) { Raise(nameof(EmergencyButtonText)); RaiseSafetyState(); } } }
    public string EmergencyButtonText => IsEmergencyStopped
        ? (SupportsRemoteEstopReset ? "复位" : "需现场复位")
        : "紧急停止";
    public bool IsLaserActive { get => _isLaserActive; private set { if (Set(ref _isLaserActive, value)) { Raise(nameof(LaserButtonText)); Raise(nameof(LaserActionHint)); ToggleLaserCommand.NotifyCanExecuteChanged(); } } }
    public bool IsRecording { get => _isRecording; private set { if (Set(ref _isRecording, value)) Raise(nameof(RecordButtonText)); } }
    public string RecordButtonText => IsRecording ? "● 录像中" : "录像";
    public string LaserButtonText => IsLaserActive ? "停止激光除锈" : "开始激光除锈";
    public string FeedbackText { get => _feedbackText; private set => Set(ref _feedbackText, value); }

    public bool IsControlSurfaceEnabled => !IsEmergencyStopped && LinkState is LinkState.Connected or LinkState.VideoLost;
    public bool IsDrivePanelEnabled => IsControlSurfaceEnabled && IsDriveMode && !_actionBusy && !ArmReturning;
    public bool IsWorkPanelEnabled => IsControlSurfaceEnabled && IsWorkMode;
    public bool CanStartLaser => false; // No real laser controller is integrated in this release.
    public string LaserActionHint
    {
        get
        {
            if (IsLaserActive) return "激光正在工作；停止操作始终可用";
            return "激光硬件未接入，参数仅供调试展示";
        }
    }

    public bool SlowSpeed { get => _slowSpeed; set { if (Set(ref _slowSpeed, value)) { Raise(nameof(SpeedGearText)); SendDrive(); } } }
    public string SpeedGearText => SlowSpeed ? "慢速精调 · 35%" : "正常行驶 · 100%";
    public double Rotation { get => _rotation; set { if (Set(ref _rotation, value)) { Raise(nameof(RotationText)); SendDrive(); } } }
    public string RotationText => $"{Rotation:+0;-0;0}%";
    public string DriveReadout
    {
        get
        {
            var magnitude = Math.Min(1, Math.Sqrt(_driveX * _driveX + _driveY * _driveY));
            if (magnitude < .03) return "方向：停止  ·  速度：0%";
            var horizontal = Math.Abs(_driveX) > .25 ? (_driveX > 0 ? "右" : "左") : "";
            var vertical = Math.Abs(_driveY) > .25 ? (_driveY > 0 ? "前" : "后") : "";
            return $"方向：{vertical}{horizontal}  ·  速度：{magnitude * (SlowSpeed ? 35 : 100):F0}%";
        }
    }
    public double FrontLeftAngle { get => _frontLeftAngle; private set => Set(ref _frontLeftAngle, value); }
    public double FrontRightAngle { get => _frontRightAngle; private set => Set(ref _frontRightAngle, value); }
    public double RearLeftAngle { get => _rearLeftAngle; private set => Set(ref _rearLeftAngle, value); }
    public double RearRightAngle { get => _rearRightAngle; private set => Set(ref _rearRightAngle, value); }

    private string _armDiagnostics = "等待机械臂诊断";
    private bool _hasArmPose;
    public string ArmDiagnostics { get => _armDiagnostics; private set => Set(ref _armDiagnostics, value); }
    public bool HasArmPose { get => _hasArmPose; private set => Set(ref _hasArmPose, value); }

    private double _jogSpeedMmS = 2.0;
    public double JogSpeedMmS
    {
        get => _jogSpeedMmS;
        set
        {
            if (!double.IsFinite(value)) return;
            if (Set(ref _jogSpeedMmS, Math.Clamp(value, 2, JogSpeedMaximum)))
            {
                StopArmJog(); // A speed change requires a fresh press, including touch.
                Raise(nameof(JogSpeedText));
            }
        }
    }
    public double JogSpeedMaximum => _robotLink is RealRobotLink r ? r.LinearSpeedLimit : 50;
    public double JogAngularMaximum => _robotLink is RealRobotLink r ? r.AngularSpeedLimit : 10;
    private double _jogAngularSpeedDegS=1;
    public double JogAngularSpeedDegS {
        get=>_jogAngularSpeedDegS;
        set { if(double.IsFinite(value) && Set(ref _jogAngularSpeedDegS,Math.Clamp(value,1,JogAngularMaximum))) {
            StopArmJog();Raise(nameof(JogAngularSpeedText));
        } }
    }
    public string JogSpeedText => $"平移目标速度 {JogSpeedMmS:F0} mm/s";
    public string JogAngularSpeedText => $"姿态目标速度 {JogAngularSpeedDegS:F0}°/s";
    private void SendScaledArmJog(float[] input)
    {
        var linear=JogSpeedMmS/JogSpeedMaximum;
        var angular=JogAngularSpeedDegS/JogAngularMaximum;
        var deadband=_robotLink is RealRobotLink r ? r.InputDeadband : .02;
        if(Math.Sqrt(input[0]*input[0]+input[1]*input[1]+input[2]*input[2])<=deadband) input[0]=input[1]=input[2]=0;
        if(Math.Sqrt(input[3]*input[3]+input[4]*input[4])<=deadband) input[3]=input[4]=0;
        // Quantize toward zero BEFORE wire rounding so diagonal components cannot
        // round up together and exceed the selected resultant velocity.
        static float WireFloor(double value)=>(float)(Math.Truncate(value*1000)/1000);
        _robotLink.SendArmJogCommand(WireFloor(input[0]*linear),WireFloor(input[1]*linear),WireFloor(input[2]*linear),WireFloor(input[3]*angular),WireFloor(input[4]*angular));
    }

    public void UpdateArmAxis(string axis)
    {
        if (!CanJogArm || _activeArmInput != "axis") return;
        var input = _directions.MapAxis(axis);
        SendScaledArmJog(input);
    }

    public void StopArmJog()
    {
        _activeArmInput = null;
        _armX = _armY = 0;
        _robotLink.SendArmJogCommand(0, 0, 0, 0, 0);
        Raise(nameof(ArmReadout));
        ArmInputCancelled?.Invoke();
    }

    public RelayCommand SaveDirectionsCommand { get; }
    public bool CanJogArm => IsWorkPanelEnabled && _backendReady && !_actionBusy && !ArmReturning;
    public string CalibrationText => _directions.Calibrated ? "方向已现场确认并保存" : "方向未校准：先启用核对模式，逐轴验证";
    public bool CommissioningMode
    {
        get => _commissioning;
        set { if (Set(ref _commissioning, value)) { StopArmJog(); Raise(nameof(CanJogArm)); } }
    }
    public bool ReverseRight { get => _directions.RightYSign < 0; set { if (value == ReverseRight) return; _directions.RightYSign = value ? -1 : 1; DirectionChanged(); Raise(); } }
    public bool ReverseUp { get => _directions.UpZSign < 0; set { if (value == ReverseUp) return; _directions.UpZSign = value ? -1 : 1; DirectionChanged(); Raise(); } }
    public bool ReverseExtend { get => _directions.ExtendXSign < 0; set { if (value == ReverseExtend) return; _directions.ExtendXSign = value ? -1 : 1; DirectionChanged(); Raise(); } }
    public bool XChecked { get => _xChecked; set { if (Set(ref _xChecked, value)) SaveDirectionsCommand.NotifyCanExecuteChanged(); } }
    public bool YChecked { get => _yChecked; set { if (Set(ref _yChecked, value)) SaveDirectionsCommand.NotifyCanExecuteChanged(); } }
    public bool ZChecked { get => _zChecked; set { if (Set(ref _zChecked, value)) SaveDirectionsCommand.NotifyCanExecuteChanged(); } }
    private void DirectionChanged()
    {
        StopArmJog();
        _directions.Calibrated = false;
        _directions.ConfirmedAtUtc = null;
        XChecked = YChecked = ZChecked = false;
        Raise(nameof(CalibrationText)); Raise(nameof(CanJogArm));
        SaveDirectionsCommand.NotifyCanExecuteChanged();
        if (_directionsPath is not null)
        {
            try { _directions.Save(_directionsPath); }
            catch (Exception ex) { FeedbackText = "方向配置未保存：" + ex.Message; }
        }
    }
    private void SaveDirections()
    {
        StopArmJog();
        if (!(XChecked && YChecked && ZChecked)) return;
        try
        {
            _directions.Calibrated = true;
            _directions.ConfirmedAtUtc = DateTime.UtcNow.ToString("O");
            if (_directionsPath is not null) _directions.Save(_directionsPath);
            CommissioningMode = false;
        }
        catch (Exception ex) { _directions.Calibrated = false; FeedbackText = "方向配置未保存：" + ex.Message; }
        Raise(nameof(CalibrationText)); Raise(nameof(CanJogArm));
        SaveDirectionsCommand.NotifyCanExecuteChanged();
    }
    public bool BeginArmInput(string source)
    {
        if (!CanJogArm || _activeArmInput is not null) return false;
        _robotLink.SetMode(OperationMode.Work);
        _activeArmInput = source;
        return true;
    }

    public double ArmHeight { get => _armHeight; private set => Set(ref _armHeight, value); }
    public double ArmPitch { get => _armPitch; private set => Set(ref _armPitch, value); }
    public double ArmYaw { get => _armYaw; private set => Set(ref _armYaw, value); }
    public string ArmReadout
    {
        get
        {
            var magnitude = Math.Min(1, Math.Sqrt(_armX * _armX + _armY * _armY));
            if (magnitude < .03) return "末端：停止  ·  力度：0%";
            var x = Math.Abs(_armX) > .25 ? (_armX > 0 ? "右" : "左") : "";
            var y = Math.Abs(_armY) > .25 ? (_armY > 0 ? "上" : "下") : "";
            return $"末端：{y}{x}  ·  力度：{magnitude:P0}";
        }
    }

    public double LaserPower { get => _laserPower; set { if (Set(ref _laserPower, value)) PushLaserParams(); } }
    public double ScanSpeed { get => _scanSpeed; set { if (Set(ref _scanSpeed, value)) PushLaserParams(); } }
    public int DebugInterlockIndex { get => _debugInterlockIndex; set => Set(ref _debugInterlockIndex, value); }

    public async Task InitializeAsync() => await _robotLink.ConnectAsync();

    public void UpdateDriveJoystick(double x, double y)
    {
        _driveX = (float)x;
        _driveY = (float)y;
        Raise(nameof(DriveReadout));
        UpdateWheelAngles();
        SendDrive();
    }

    public void UpdateArmJoystick(double x, double y)
    {
        if (!CanJogArm || _activeArmInput != "joystick") return;
        _armX = (float)x;
        _armY = (float)y;
        Raise(nameof(ArmReadout));
        var input = _directions.MapJoystick(x, y);
        SendScaledArmJog(input);
    }

    private void HandleTelemetry(TelemetrySnapshot telemetry)
    {
        BatteryPercent = telemetry.BatteryPercent; Raise(nameof(BatteryDisplay)); Raise(nameof(BaseAttitudeDisplay));
        LatencyMs = telemetry.LatencyMs;
        RollDeg = telemetry.RollDeg;
        PitchDeg = telemetry.PitchDeg;
        for (var i = 0; i < Math.Min(6, telemetry.JointMargins.Length); i++) JointMargins[i].Value = telemetry.JointMargins[i];
        for (var i = 0; i < Math.Min(5, telemetry.InterlockStatus.Length); i++) Interlocks[i].IsSatisfied = telemetry.InterlockStatus[i];
        // An active laser is fail-safe: a newly failed robot-side interlock issues an immediate stop.
        var robotInterlocksSatisfied = telemetry.InterlockStatus.Take(5).All(value => value);
        if (telemetry.LaserActive && !robotInterlocksSatisfied)
        {
            _robotLink.SendLaserCommand(false);
            IsLaserActive = false;
        }
        else
        {
            IsLaserActive = telemetry.LaserActive;
        }
        Raise(nameof(MinimumJointText));
        Raise(nameof(IsMinimumJointWarning));
        RaiseSafetyState();
    }

    private void HandleRustSpots(List<RustSpot> spots)
    {
        var selectedId = SelectedSpot?.Id;
        RustSpots.Clear();
        foreach (var spot in spots) RustSpots.Add(new RustSpotViewModel(spot));
        if (selectedId.HasValue) SelectSpot(RustSpots.FirstOrDefault(x => x.Id == selectedId.Value));
    }

    private void HandleLinkState(LinkState state)
    {
        // Video loss still leaves the control path alive, so explicitly stop before locking the laser.
        // On ControlLost no regular command is attempted; the robot's local safety mode owns the stop.
        if (state == LinkState.VideoLost && IsLaserActive)
            _robotLink.SendLaserCommand(false);
        if (state != LinkState.Connected) StopArmJog();
        LinkState = state;
        if (state == LinkState.ControlLost && Mode != OperationMode.Standby)
            Mode = OperationMode.Standby;
        Interlocks[5].IsSatisfied = state != LinkState.VideoLost && state != LinkState.ControlLost;
        if (state != LinkState.Connected) IsLaserActive = false;
        RaiseSafetyState();
    }

    private void SwitchMode(OperationMode mode)
    {
        if (!CanChangeMode()) return;
        StopArmJog();
        if (IsLaserActive) { _robotLink.SendLaserCommand(false); IsLaserActive = false; }
        _driveX = _driveY = 0;
        _armX = _armY = 0;
        Rotation = 0;
        _robotLink.SendDriveCommand(0, 0, 0);
        _robotLink.SendArmJogCommand(0, 0, 0, 0, 0);
        Raise(nameof(ArmReadout));
        Mode = mode;
        _robotLink.SetMode(mode);
        _robotLink.SetVideoQualityMode(mode == OperationMode.Work ? VideoQualityMode.ClarityPriority : VideoQualityMode.SmoothPriority);
        FeedbackText = mode switch { OperationMode.Drive => "已切换：流畅优先", OperationMode.Work => "已切换：清晰优先", _ => "已进入待机" };
        StartFeedbackTimer();
    }

    private void SetDriveSubmode(DriveSubmode mode)
    {
        DriveSubmode = mode;
        _robotLink.SetDriveSubmode(mode);
        UpdateWheelAngles();
    }

    private void UpdateWheelAngles()
    {
        if (DriveSubmode == DriveSubmode.Spin)
        {
            FrontLeftAngle = 45; FrontRightAngle = -45; RearLeftAngle = -45; RearRightAngle = 45;
            return;
        }
        var angle = Math.Abs(_driveX) + Math.Abs(_driveY) < .03 ? 0 : Math.Atan2(_driveX, _driveY) * 180 / Math.PI;
        FrontLeftAngle = FrontRightAngle = angle;
        RearLeftAngle = RearRightAngle = DriveSubmode == DriveSubmode.Crab ? angle : 0;
    }

    private void SendDrive()
    {
        if (!IsDrivePanelEnabled) return;
        var factor = SlowSpeed ? .35f : 1f;
        _robotLink.SendDriveCommand(_driveX * factor, _driveY * factor, (float)(Rotation / 100) * factor);
    }

    private void AdjustHeight(double delta)
    {
        var next = Math.Clamp(ArmHeight + delta, 0, 300);
        var actualDelta = next - ArmHeight;
        ArmHeight = next;
        if (actualDelta != 0) _robotLink.SetArmHeight((float)actualDelta);
    }

    private void AdjustPitch(double delta)
    {
        ArmPitch = Math.Clamp(ArmPitch + delta, -90, 90);
        SendScaledArmJog([0, 0, 0, (float)(delta / 5), 0]);
    }

    private void AdjustYaw(double delta)
    {
        ArmYaw = Math.Clamp(ArmYaw + delta, -90, 90);
        SendScaledArmJog([0, 0, 0, 0, (float)(delta / 5)]);
    }

    private void ArmHome()
    {
        ArmHeight = 150; ArmPitch = 0; ArmYaw = 0;
        _robotLink.ArmHome();
        FeedbackText = "机械臂已收回安全位";
        StartFeedbackTimer();
    }

    private void ApplyPreset(string preset, double power, double speed)
    {
        LaserPower = power; ScanSpeed = speed;
        _robotLink.SetLaserPreset(preset);
    }

    private void PushLaserParams()
    {
        if (CanUseWorkControls()) _robotLink.SetLaserParams((float)LaserPower, (float)ScanSpeed);
    }

    private void ToggleLaser()
    {
        // Stopping is always allowed. Starting is guarded by mode, six interlocks and connection state.
        if (IsLaserActive)
        {
            _robotLink.SendLaserCommand(false);
            IsLaserActive = false;
        }
        else if (CanStartLaser)
        {
            _robotLink.SendLaserCommand(true);
            IsLaserActive = true;
        }
    }

    private void ToggleEmergency()
    {
        if (!IsEmergencyStopped)
        {
            // Stop locally first; no regular command is sent after this safety boundary.
            StopArmJog();
            _robotLink.SendEstop();
            IsLaserActive = false;
            IsEmergencyStopped = true;
            _driveX = _driveY = 0; _rotation = 0; _armX = _armY = 0;
            Raise(nameof(DriveReadout)); Raise(nameof(ArmReadout)); Raise(nameof(Rotation)); Raise(nameof(RotationText));
        }
        else if (CanResetEmergency)
        {
            // The supplied protocol has no reset message. The mock therefore models the explicit operator reset locally.
            IsEmergencyStopped = false;
        }
    }

    private void Snapshot()
    {
        _robotLink.SendSnapshot();
        FeedbackText = "✓ 已拍照";
        StartFeedbackTimer();
    }

    private void ToggleRecord()
    {
        IsRecording = !IsRecording;
        _robotLink.SendRecordToggle(IsRecording);
    }

    private async Task ReconnectAsync()
    {
        if (IsRecording) IsRecording = false;
        await _robotLink.ConnectAsync();
    }

    private void SelectSpot(RustSpotViewModel? spot)
    {
        foreach (var item in RustSpots) item.IsSelected = ReferenceEquals(item, spot);
        SelectedSpot = spot;
    }

    private void ToggleDebugInterlock()
    {
        if (_debugLink is null || DebugInterlockIndex < 0 || DebugInterlockIndex > 4) return;
        _debugLink.SetInterlock(DebugInterlockIndex, !Interlocks[DebugInterlockIndex].IsSatisfied);
    }

    private bool CanSendNormalControl() => IsControlSurfaceEnabled;
    private bool CanChangeMode() => IsControlSurfaceEnabled && !_actionBusy && !ArmReturning;
    private bool CanUseArm() => IsWorkPanelEnabled;
    private bool CanUseWorkControls() => IsWorkPanelEnabled && !_actionBusy && !ArmReturning;

    private void RaiseModeState()
    {
        Raise(nameof(IsStandbyMode)); Raise(nameof(IsDriveMode)); Raise(nameof(IsWorkMode)); Raise(nameof(ModeText));
        RaiseSafetyState();
    }

    private void RaiseDriveSubmodeState()
    {
        Raise(nameof(IsStandardDrive)); Raise(nameof(IsCrabDrive)); Raise(nameof(IsSpinDrive));
    }

    private void RaiseSafetyState()
    {
        Raise(nameof(ArmBlockHint));
        Raise(nameof(LinkStateText)); Raise(nameof(IsVideoLost)); Raise(nameof(IsControlLost)); Raise(nameof(IsReconnecting)); Raise(nameof(IsLinkConnected));
        Raise(nameof(CanJogArm));
        Raise(nameof(IsControlSurfaceEnabled)); Raise(nameof(IsDrivePanelEnabled)); Raise(nameof(IsWorkPanelEnabled));
        Raise(nameof(CanStartLaser)); Raise(nameof(LaserActionHint));
        foreach (var command in AllGuardedCommands()) command.NotifyCanExecuteChanged();
    }

    private IEnumerable<RelayCommand> AllGuardedCommands()
    {
        yield return EnableArmCommand; yield return RecoverArmCommand; yield return SetHomeCommand; yield return ReturnHomeCommand; yield return StopHomeCommand;
        yield return SwitchToStandbyCommand; yield return SwitchToDriveCommand; yield return SwitchToWorkCommand;
        yield return SetStandardDriveCommand; yield return SetCrabDriveCommand; yield return SetSpinDriveCommand; yield return ToggleSpeedCommand;
        yield return HeightUpCommand; yield return HeightDownCommand; yield return PitchUpCommand; yield return PitchDownCommand;
        yield return YawUpCommand; yield return YawDownCommand; yield return ArmHomeCommand;
        yield return LightPresetCommand; yield return MediumPresetCommand; yield return HeavyPresetCommand;
        yield return ToggleLaserCommand; yield return ToggleEmergencyCommand; yield return SnapshotCommand; yield return ToggleRecordCommand; yield return ReconnectCommand;
    }

    private void StartFeedbackTimer() { _feedbackTimer.Stop(); _feedbackTimer.Start(); }

    public void Dispose()
    {
        StopArmJog();
        _feedbackTimer.Stop();
        _uiHeartbeat.Stop();
        _robotLink.Disconnect();
        if (_robotLink is IDisposable disposable) disposable.Dispose();
    }
}

