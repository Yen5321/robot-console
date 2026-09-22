using System.Windows;
using System.Windows.Threading;
using RustRemoval.RobotConsole.Controls;

static class Program
{
 [STAThread] static int Main()
 {
    var app=new Application();
    var video=new WebRtcView {Active=true,StreamUrl="http://127.0.0.1:8889/robot/"};
    var window=new Window {Title="本机 WebRTC 视频测试（不连接机械臂）",Content=video,Width=680,Height=540,ShowActivated=false};
    int frames=0;bool passed=false;
    var timeout=new DispatcherTimer {Interval=TimeSpan.FromSeconds(25)};
    timeout.Tick+=(_,_)=>{Console.WriteLine("FAIL: WebRTC did not present fresh video frames");window.Close();};
    video.FramePresented+=()=> {if(++frames>=5) {passed=true;Console.WriteLine("PASS: WPF WebView2 WebRTC presented five fresh H.264 video frames");window.Close();}};
    window.Closed+=(_,_)=>{timeout.Stop();video.Close();app.Shutdown();};
    timeout.Start();app.Run(window);return passed?0:1;
 }
}
