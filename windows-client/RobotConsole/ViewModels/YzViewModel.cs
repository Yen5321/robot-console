using System.Windows.Media.Imaging;
using RustRemoval.RobotConsole.Models;
using RustRemoval.RobotConsole.Services;
namespace RustRemoval.RobotConsole.ViewModels;

public sealed class YzViewModel : ObservableObject, IDisposable
{
    private readonly RealRobotLink _link;
    private readonly JogDirections _directions;
    private ArmReport _report = new(false,false,false,null,"使能未知",null,false,false,"");
    private LinkState _state = LinkState.Reconnecting;
    private bool _busy, _pressed;
    private string _notice="连接后使能即可点动，无需方向校准勾选";
    private string _rtt="--", _loss="--", _pose="XYZ / RPY：等待反馈", _input="松开：停止";
    private BitmapSource? _video;
    public event Action? CancelInput;
    public YzViewModel(RealRobotLink link, JogDirections? directions=null)
    {
        _link=link; _directions=directions ?? new JogDirections();
        EnableCommand=new RelayCommand(async()=>await Action("enable"),()=>CanAction && !_report.ReturningHome);
        RecoverCommand=new RelayCommand(async()=>await Action("recover"),()=>CanAction && (_report.Faulted || _report.Status==1));
        SetHomeCommand=new RelayCommand(async()=>await Action("set_home"),()=>CanAction && _report.Ready && !_report.ReturningHome);
        HomeCommand=new RelayCommand(async()=>await Action("home"),()=>CanAction && _report.Ready && _report.HasHome && !_report.ReturningHome);
        StopHomeCommand=new RelayCommand(async()=>await Action("stop"),()=>CanAction && _report.ReturningHome);
        EstopCommand=new RelayCommand(()=> { StopInput(); _link.SendEstop(); _report=_report with {Ready=false,Faulted=true}; Notice="急停已发送"; Refresh(); });
        _link.OnArmReport+=ApplyReport;
        _link.OnVideoFrame+=frame=>Video=frame;
        _link.OnLinkStateChanged+=ApplyConnection;
        _link.OnNetworkMetrics+=m=> {
            Rtt=m.RttMs is double r ? $"{r:F0} ms" : "--";
            Loss=m.AckTimeoutPercent is double p ? $"{p:F1}%（{m.TimedOut}/{m.Completed}）" : "采样中";
        };
    }
    public RelayCommand EnableCommand {get;}
    public RelayCommand RecoverCommand {get;}
    public RelayCommand SetHomeCommand {get;}
    public RelayCommand HomeCommand {get;}
    public RelayCommand StopHomeCommand {get;}
    public RelayCommand EstopCommand {get;}
    public bool CanAction=>!_busy && _report.Compatible && _state is LinkState.Connected or LinkState.VideoLost;
    public bool CanJog=>CanAction && _report.Ready && !_report.Faulted && !_report.ReturningHome;
    public string Status=> $"{_report.Enabled} · 状态 {_report.Status?.ToString() ?? "--"}";
    public string Connection=>_state switch { LinkState.Connected=>"控制与视频已连接", LinkState.VideoLost=>"视频中断", LinkState.ControlLost=>"控制中断", _=>"正在连接" };
    public string Error=>_report.Error;
    public string HomeStatus=>_report.ReturningHome ? "正在低速回到记录零位" : _report.HasHome ? "零位已记录" : "尚未记录零位";
    public string Notice {get=>_notice;private set=>Set(ref _notice,value);}
    public string Rtt {get=>_rtt;private set=>Set(ref _rtt,value);}
    public string Loss {get=>_loss;private set=>Set(ref _loss,value);}
    public string Pose {get=>_pose;private set=>Set(ref _pose,value);}
    public string Input {get=>_input;private set=>Set(ref _input,value);}
    public BitmapSource? Video {get=>_video;private set=>Set(ref _video,value);}
    public async Task InitializeAsync()=>await _link.ConnectAsync();
    public void ApplyConnection(LinkState state) { if(state!=LinkState.Connected) StopInput(); _state=state; Refresh(); }
    public void ApplyReport(ArmReport report)
    {
        if(!report.Ready || report.Faulted) StopInput();
        _report=report;
        Pose=report.Pose is {Length:6} p ? $"基座 XYZ：{p[0]:F1} / {p[1]:F1} / {p[2]:F1} mm\nRPY：{p[3]:F1} / {p[4]:F1} / {p[5]:F1}°" : "XYZ / RPY：反馈不可用";
        Refresh();
    }
    public bool BeginInput()
    {
        if(!CanJog || _pressed) return false;
        _pressed=true; _link.SetMode(OperationMode.Work); return true;
    }
    public void UpdateInput(double x,double y)
    {
        if(!_pressed || !CanJog) return;
        var mapped=_directions.MapJoystick(x,y);
        _link.SendArmJogCommand(0,mapped[1],mapped[2],0,0);
        Input=$"Y {mapped[1]:+0.00;-0.00;0.00} · Z {mapped[2]:+0.00;-0.00;0.00}";
    }
    public void StopInput()
    {
        _pressed=false;
        _link.SendArmJogCommand(0,0,0,0,0);
        Input="松开：保持当前位置";
        CancelInput?.Invoke();
    }
    private async Task Action(string action)
    {
        if(!CanAction) return;
        StopInput(); _busy=true; Refresh(); Notice="正在执行…";
        try { await _link.ExecuteArmActionAsync(action);
            Notice=action switch {"recover"=>"急停已恢复并确认使能；重新按下摇杆才运动", "enable"=>"使能已确认", "set_home"=>"当前位姿已记录为零位", "home"=>"回零已开始，可按停止回零", _=>"已下发当前位置保持"}; }
        catch(Exception ex) { Notice="操作未完成："+ex.Message; }
        finally { _busy=false; Refresh(); }
    }
    private void Refresh()
    {
        Raise(nameof(CanAction));Raise(nameof(CanJog));Raise(nameof(Status));Raise(nameof(Connection));Raise(nameof(Error));Raise(nameof(HomeStatus));
        foreach(var c in new[]{EnableCommand,RecoverCommand,SetHomeCommand,HomeCommand,StopHomeCommand}) c.NotifyCanExecuteChanged();
    }
    public void Dispose() { StopInput();_link.Dispose(); }
}
