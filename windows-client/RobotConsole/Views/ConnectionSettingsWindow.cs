using System.IO;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using RustRemoval.RobotConsole.Services;

namespace RustRemoval.RobotConsole.Views;

public sealed class ConnectionSettingsWindow : Window
{
    private readonly Dictionary<string,TextBox> _fields=new();
    private readonly string _path;
    private readonly RobotLinkSettings _settings;
    private readonly TextBlock _error=new() { Foreground=Brushes.OrangeRed,TextWrapping=TextWrapping.Wrap,Margin=new Thickness(0,10,0,5) };
    private readonly ComboBox _video=new() { ItemsSource=new[]{"WebRTC","MJPEG"},MinHeight=40,Foreground=Brushes.Black };
    public ConnectionSettingsWindow(string path)
    {
        _path=path;
        try { _settings=RobotLinkSettings.Load(path); }
        catch { _settings=new RobotLinkSettings {Mode="Real",RobotHost="10.126.122.113"}; }
        Title="v8.1 · 连接设置（已断开控制）";Width=680;Height=760;MinWidth=540;MinHeight=540;
        WindowStartupLocation=WindowStartupLocation.CenterOwner;
        Background=new SolidColorBrush(Color.FromRgb(18,28,37));Foreground=Brushes.White;
        var panel=new StackPanel { Margin=new Thickness(24) };
        Content=new ScrollViewer {Content=panel,VerticalScrollBarVisibility=ScrollBarVisibility.Auto};
        panel.Children.Add(new TextBlock {Text="保存后使用新连接，速度恢复默认低速；不会自动恢复急停或使能。",TextWrapping=TextWrapping.Wrap,Margin=new Thickness(0,0,0,12)});
        Add(panel,"host","机器人 IP / 主机名",_settings.RobotHost);
        Add(panel,"udp","UDP 端口",_settings.UdpPort.ToString());
        var oldUri=Uri.TryCreate(_settings.MjpegUrl,UriKind.Absolute,out var u)?u:new Uri("http://10.126.122.113:8080/stream.mjpg");
        Add(panel,"http","主链路 HTTP 端口",oldUri.Port.ToString());
        Add(panel,"local","主链路本机网卡 IP（留空为自动）",_settings.LocalBindAddress);
        panel.Children.Add(new TextBlock {Text="视频协议",Margin=new Thickness(0,10,0,3)});
        var itemText=new FrameworkElementFactory(typeof(TextBlock));
        itemText.SetValue(TextBlock.ForegroundProperty,Brushes.Black);
        itemText.SetBinding(TextBlock.TextProperty,new System.Windows.Data.Binding());
        _video.ItemTemplate=new DataTemplate {VisualTree=itemText};
        _video.SelectedItem=_settings.VideoTransport;panel.Children.Add(_video);
        Add(panel,"video","主链路 WebRTC 地址",_settings.WebRtcUrl);
        Add(panel,"mjpeg","主链路 MJPEG 路径（HTTP 主机跟随机器人地址）",oldUri.PathAndQuery);
        Add(panel,"backup","备用机器人 IP（不用则留空）",_settings.BackupRobotHost);
        Add(panel,"backupHttp","备用 HTTP 基地址",_settings.BackupHttpUrl);
        Add(panel,"backupVideo","备用 WebRTC 地址",_settings.BackupWebRtcUrl);
        Add(panel,"backupLocal","备用本机网卡 IP",_settings.BackupLocalBindAddress);
        panel.Children.Add(_error);
        var save=new Button {Content=new TextBlock {Text="保存并连接",Foreground=Brushes.Black},MinHeight=48,Margin=new Thickness(0,12,0,6)};
        save.Click+=(_,_)=>Save();panel.Children.Add(save);
        var cancel=new Button {Content=new TextBlock {Text="取消并重新连接原配置",Foreground=Brushes.Black},MinHeight=44};cancel.Click+=(_,_)=>DialogResult=false;panel.Children.Add(cancel);
    }
    private void Add(Panel panel,string key,string label,string? value)
    {
        panel.Children.Add(new TextBlock {Text=label,Margin=new Thickness(0,10,0,3)});
        var box=new TextBox {Text=value??"",Foreground=Brushes.Black,MinHeight=40,Padding=new Thickness(8),FontSize=16,VerticalContentAlignment=VerticalAlignment.Center};
        _fields[key]=box;panel.Children.Add(box);
    }
    private string Text(string key)=>_fields[key].Text.Trim();
    private string? Optional(string key)=>string.IsNullOrWhiteSpace(Text(key))?null:Text(key);
    private void Save()
    {
        try {
            var next=JsonSerializer.Deserialize<RobotLinkSettings>(JsonSerializer.Serialize(_settings))!;
            next.Mode="Real";next.RobotHost=Text("host");next.UdpPort=int.Parse(Text("udp"));
            var http=int.Parse(Text("http"));if(http<1||http>65535) throw new InvalidOperationException("HTTP 端口应为 1–65535");
            var path=Text("mjpeg");if(!path.StartsWith('/')) throw new InvalidOperationException("MJPEG 路径应以 / 开头");
            next.MjpegUrl=new UriBuilder("http",next.RobotHost,http){Path=path.Split('?')[0],Query=path.Contains('?')?path[(path.IndexOf('?')+1)..]:""}.Uri.AbsoluteUri;
            next.VideoTransport=(string?)_video.SelectedItem??"WebRTC";next.WebRtcUrl=Text("video");
            next.LocalBindAddress=Optional("local");next.BackupRobotHost=Optional("backup");
            next.BackupHttpUrl=next.BackupRobotHost is null?null:Optional("backupHttp");
            next.BackupWebRtcUrl=next.BackupRobotHost is null?null:Optional("backupVideo");
            next.BackupLocalBindAddress=next.BackupRobotHost is null?null:Optional("backupLocal");
            next.Save(_path);DialogResult=true;
        } catch(Exception ex) { _error.Text="配置未保存："+ex.Message; }
    }
}
