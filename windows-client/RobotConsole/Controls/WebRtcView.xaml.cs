using System.Windows;
using System.Windows.Controls;
using Microsoft.Web.WebView2.Core;
namespace RustRemoval.RobotConsole.Controls;

public partial class WebRtcView:UserControl
{
    public static readonly DependencyProperty StreamUrlProperty=DependencyProperty.Register(nameof(StreamUrl),typeof(string),typeof(WebRtcView),new PropertyMetadata("",Changed));
    public static readonly DependencyProperty ActiveProperty=DependencyProperty.Register(nameof(Active),typeof(bool),typeof(WebRtcView),new PropertyMetadata(false,Changed));
    public string StreamUrl {get=>(string)GetValue(StreamUrlProperty);set=>SetValue(StreamUrlProperty,value);}
    public bool Active {get=>(bool)GetValue(ActiveProperty);set=>SetValue(ActiveProperty,value);}
    public event Action? FramePresented;
    private bool _initializing,_ready;
    public WebRtcView() { InitializeComponent();Loaded+=(_,_)=>Navigate(); }
    private static void Changed(DependencyObject d,DependencyPropertyChangedEventArgs e)=>((WebRtcView)d).Navigate();
    private bool Allowed(string source)=>Uri.TryCreate(source,UriKind.Absolute,out var a)&&Uri.TryCreate(StreamUrl,UriKind.Absolute,out var b)&&a.Scheme==b.Scheme&&a.Host==b.Host&&a.Port==b.Port;
    private async void Navigate()
    {
        if(!IsLoaded||!Active||string.IsNullOrWhiteSpace(StreamUrl)||_initializing) return;
        try {
            if(!_ready) {
                _initializing=true;
                await Browser.EnsureCoreWebView2Async();
                Browser.CoreWebView2.Settings.AreDefaultContextMenusEnabled=false;
                Browser.CoreWebView2.Settings.AreDevToolsEnabled=false;
                Browser.CoreWebView2.Settings.IsStatusBarEnabled=false;
                Browser.CoreWebView2.NewWindowRequested+=(_,e)=>e.Handled=true;
                Browser.CoreWebView2.PermissionRequested+=(_,e)=>e.State=CoreWebView2PermissionState.Deny;
                Browser.CoreWebView2.DownloadStarting+=(_,e)=>e.Cancel=true;
                Browser.CoreWebView2.NavigationStarting+=(_,e)=> { if(!Allowed(e.Uri)) e.Cancel=true; };
                Browser.CoreWebView2.WebMessageReceived+=(_,e)=> { if(Allowed(e.Source) && e.TryGetWebMessageAsString()=="robot-video-frame") { ErrorText.Text="";FramePresented?.Invoke(); } };
                await Browser.CoreWebView2.AddScriptToExecuteOnDocumentCreatedAsync("""
                    (()=>{
                      let last=0;
                      const watch=()=>{const v=document.querySelector('video');if(!v||v.dataset.robotWatching)return;
                        v.dataset.robotWatching='1';v.muted=true;
                        const frame=(now)=>{if(now-last>150){window.chrome.webview.postMessage('robot-video-frame');last=now;}v.requestVideoFrameCallback(frame);};
                        if(v.requestVideoFrameCallback)v.requestVideoFrameCallback(frame);
                      };setInterval(watch,300);
                    })();
                    """);
                _ready=true;
            }
            ErrorText.Text="正在连接 WebRTC 视频…";
            Browser.CoreWebView2.Navigate(StreamUrl);
        } catch(Exception ex) { ErrorText.Text="WebRTC 未就绪："+ex.Message+"\n请检查 WebView2 Runtime 与机器人视频服务"; }
        finally {_initializing=false;}
    }
    public void Close()=>Browser.Dispose();
}
