using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using RustRemoval.RobotConsole.ViewModels;

namespace RustRemoval.RobotConsole.Views;

public sealed class DiagnosticsWindow : Window
{
    public DiagnosticsWindow(MainViewModel vm)
    {
        Title="v8.1 · 详细诊断 / 最近日志";Width=900;Height=720;WindowStartupLocation=WindowStartupLocation.CenterOwner;
        var root=new DockPanel {Margin=new Thickness(12)};Content=root;
        var bar=new StackPanel {Orientation=Orientation.Horizontal};DockPanel.SetDock(bar,Dock.Top);root.Children.Add(bar);
        var box=new TextBox {IsReadOnly=true,AcceptsReturn=true,TextWrapping=TextWrapping.NoWrap,VerticalScrollBarVisibility=ScrollBarVisibility.Auto,HorizontalScrollBarVisibility=ScrollBarVisibility.Auto,FontFamily=new FontFamily("Consolas"),FontSize=13};
        root.Children.Add(box);
        string logs="点击刷新读取机器人最近日志。完整历史位于机器人 logs/jog-v5.jsonl。";
        void Refresh()=>box.Text=vm.DiagnosticDetails+"\n\n=== 最近日志 ===\n"+logs;
        var refresh=new Button {Content="读取最近日志",MinHeight=44,Padding=new Thickness(12,5,12,5)};
        refresh.Click+=async (_,_)=> {refresh.IsEnabled=false;try {logs=await vm.RecentDiagnosticLogAsync();}catch(Exception ex){logs=ex.Message;}finally {Refresh();refresh.IsEnabled=true;}};
        bar.Children.Add(refresh);
        var export=new Button {Content="导出诊断",MinHeight=44,Margin=new Thickness(12,0,0,0),Padding=new Thickness(12,5,12,5)};
        export.Click+=(_,_)=>{var dialog=new Microsoft.Win32.SaveFileDialog {FileName="robot-v8-diagnostics.txt",Filter="文本|*.txt"};if(dialog.ShowDialog(this)==true) {try {File.WriteAllText(dialog.FileName,box.Text);}catch(Exception ex){MessageBox.Show(this,ex.Message,"导出失败");}}};
        bar.Children.Add(export);
        var timer=new DispatcherTimer {Interval=TimeSpan.FromMilliseconds(500)};timer.Tick+=(_,_)=>Refresh();
        Loaded+=(_,_)=>{Refresh();timer.Start();};Closed+=(_,_)=>timer.Stop();
    }
}
