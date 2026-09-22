using System.IO;
using System.Reflection;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using RustRemoval.RobotConsole;
using RustRemoval.RobotConsole.Controls;
using RustRemoval.RobotConsole.Services;
using RustRemoval.RobotConsole.ViewModels;
using RustRemoval.RobotConsole.Views;
using RustRemoval.RobotConsole.Models;

static class Program
{
    [STAThread]
    static void Main(string[] args)
    {
        // Offline rendering: do not Show/Run the window or initialize a real link.
        var app = new App(); app.InitializeComponent();
        var link = new RealRobotLink(new RobotLinkSettings { Mode="Real", RobotHost="127.0.0.1", MjpegUrl="http://127.0.0.1:28085/stream.mjpg" });
        var vm = new MainViewModel(link);
        typeof(MainViewModel).GetMethod("HandleLinkState",BindingFlags.NonPublic|BindingFlags.Instance)!.Invoke(vm,[LinkState.Connected]);
        vm.ApplyArmReport(new(true,true,false,0,"六轴已使能",[63.2,-.6,169.6,-148.8,60.3,-146.7],true,false,""));
        vm.SwitchToWorkCommand.Execute(null);
        Assert(vm.CanJogArm, "enabled healthy arm can jog without calibration checklist");
        var controlField=typeof(RealRobotLink).GetField("_control",BindingFlags.NonPublic|BindingFlags.Instance)!;
        RobotControlState CurrentControl() => (RobotControlState)controlField.GetValue(link)!;
        vm.JogSpeedMmS=10;
        Assert(vm.BeginArmInput("joystick"), "speed test press");
        vm.UpdateArmJoystick(1,1);
        var slow=CurrentControl();
        Assert(Math.Sqrt(slow.ArmDy*slow.ArmDy+slow.ArmDz*slow.ArmDz)*50<=10 && Math.Sqrt(slow.ArmDy*slow.ArmDy+slow.ArmDz*slow.ArmDz)*50>9.9, "diagonal full joystick respects selected translation speed after wire quantization");
        vm.JogSpeedMmS=4;
        Assert(CurrentControl().ArmDy==0 && CurrentControl().ArmDz==0, "speed change clears held input");
        vm.UpdateArmJoystick(1,0);
        Assert(CurrentControl().ArmDy==0, "speed change requires fresh press");
        vm.JogAngularSpeedDegS=10;
        Assert(vm.BeginArmInput("axis"), "axis speed test press");
        vm.UpdateArmAxis("pitch+");
        Assert(CurrentControl().ArmPitch==1, "maximum speed retains angular ceiling");
        vm.JogSpeedMmS=2;
        Assert(vm.BeginArmInput("axis"), "slow axis press");
        vm.UpdateArmAxis("x+");
        Assert(Math.Abs(CurrentControl().ArmDx-.04)<.0001, "extend button respects speed slider");
        Assert(vm.JogAngularSpeedDegS==10,"translation slider does not change angular speed");
        vm.JogAngularSpeedDegS=1;
        Assert(CurrentControl().ArmDx==0,"angular speed change clears translation input");
        Assert(vm.SetHomeCommand.CanExecute(null), "fresh CAN feedback enables set home");
        vm.ApplyArmReport(new(true,false,false,0,"六轴已使能",null,false,false,"示教模式"));
        Assert(!vm.CanJogArm && !vm.SetHomeCommand.CanExecute(null) && vm.EnableArmCommand.CanExecute(null), "teach mode blocks motion but leaves explicit CAN enable reachable");
        vm.ApplyArmReport(new(true,true,false,0,"六轴已使能",null,true,false,""));
        Assert(vm.BeginArmInput("joystick"), "healthy YZ press starts input");
        vm.StopArmJog();
        vm.UpdateArmJoystick(1,1);
        Assert(vm.ArmReadout.Contains("停止"), "stale pointer movement after release does not restart");
        foreach(var selected in new[]{2,4,10,20,50}) {
            vm.JogSpeedMmS=selected;
            for(int angle=0;angle<360;angle+=7) {
                vm.StopArmJog();vm.BeginArmInput("joystick");vm.UpdateArmJoystick(Math.Cos(angle*Math.PI/180),Math.Sin(angle*Math.PI/180));
                var packet=WireV6.Control(1,1,1,true,CurrentControl());
                double y=System.Buffers.Binary.BinaryPrimitives.ReadInt16LittleEndian(packet.AsSpan(19))/1000.0;
                double z=System.Buffers.Binary.BinaryPrimitives.ReadInt16LittleEndian(packet.AsSpan(21))/1000.0;
                if(Math.Sqrt(y*y+z*z)*50>selected+1e-7) throw new Exception("Encoded velocity exceeded selected limit");
            }
        }
        Assert(true,"encoded diagonal norm stays below selected speed at all trial levels and directions");
        vm.StopArmJog();vm.JogSpeedMmS=2;vm.JogAngularSpeedDegS=1;
        var window = new MainWindow(vm) { WindowState=WindowState.Normal, Width=1500, Height=940 };
        var joystick = (VirtualJoystick)window.FindName("ArmJoystick");
        var speedSlider=(Slider)window.FindName("JogSpeedSlider");
        var angularSlider=(Slider)window.FindName("JogAngularSpeedSlider");
        Assert(vm.VisibleInterlocks.Count()==4 && !vm.VisibleInterlocks.Any(i=>i.Name is "防护挡板到位" or "作业区域无人员"),"only requested interlock rows removed");
        Assert(!vm.CanStartLaser,"placeholder laser cannot actuate");
        int ended=0, zeros=0;
        joystick.InteractionEnded += () => ended++;
        joystick.ValueChanged += (_, e) => { if(e.X==0 && e.Y==0) zeros++; };
        var dragging = typeof(VirtualJoystick).GetField("_dragging",BindingFlags.NonPublic|BindingFlags.Instance)!;
        dragging.SetValue(joystick,true);
        joystick.CancelInput();
        Assert(ended==1 && zeros==1 && !(bool)dragging.GetValue(joystick)!, "capture cancellation emits zero and ends interaction once");
        joystick.CancelInput();
        Assert(ended==1 && zeros==1, "repeated capture cancellation is idempotent");
        dragging.SetValue(joystick,true);
        joystick.IsEnabled=false;
        Assert(ended==2 && zeros==2 && !(bool)dragging.GetValue(joystick)!, "disabled control cancels a held pointer");
        joystick.ClearValue(UIElement.IsEnabledProperty);
        joystick.SetBinding(UIElement.IsEnabledProperty, new System.Windows.Data.Binding("CanJogArm"));

        var panel=(FrameworkElement)window.Content;
        panel.Measure(new Size(1500,900)); panel.Arrange(new Rect(0,0,1500,900)); panel.UpdateLayout();
        Assert(speedSlider.Minimum==2 && speedSlider.Maximum==50, "speed slider range rendered");
        Assert(angularSlider.Minimum==1 && angularSlider.Maximum==10,"angular slider range rendered");
        var buttons=Descendants(panel).OfType<Button>().ToArray();
        Assert(new[]{"使能 / CAN 控制","恢复急停","设为零位","回到记录零位","停止回零","紧急停止"}.All(name=>buttons.Any(b=>Equals(b.Content,name))), "explicit recovery and home actions coexist with restored layout");
        Assert(!Descendants(panel).OfType<CheckBox>().Any(), "calibration checkboxes removed");
        Assert(new[]{"行驶模式","作业模式","轻度锈","拍照","伸出"}.All(name=>buttons.Any(b=>Equals(b.Content,name))), "original drive, laser, camera and axis controls restored");
        vm.ApplyArmReport(new(true,false,true,1,"六轴已使能",null,true,false,"急停锁存"));
        Assert(!vm.CanJogArm && vm.RecoverArmCommand.CanExecute(null), "estop blocks jog but exposes explicit recovery");
        panel.UpdateLayout();
        Assert(buttons.Single(b=>Equals(b.Content,"恢复急停")).IsEnabled, "recovery button has no disabled ancestor during estop");
        vm.ApplyArmReport(new(true,true,false,0,"六轴已使能",[63.2,-.6,169.6,-148.8,60.3,-146.7],true,false,""));
        typeof(MainViewModel).GetProperty("ArmDiagnostics")!.SetValue(vm,"当前异常：无（软件预览）\n使能状态：六轴已使能\n控制模式：CAN 控制\nXYZ 63.2 / -0.6 / 169.6 mm\nRPY -148.8 / 60.3 / -146.7°");
        typeof(MainViewModel).GetProperty("HasArmPose")!.SetValue(vm,true);
        panel.UpdateLayout();
        if(args.Length>0)
        {
            var bmp=new RenderTargetBitmap(1500,900,96,96,PixelFormats.Pbgra32); bmp.Render(panel);
            var encoder=new PngBitmapEncoder(); encoder.Frames.Add(BitmapFrame.Create(bmp));
            using var file=File.Create(args[0]); encoder.Save(file);
        }
        panel.Measure(new Size(1180,720)); panel.Arrange(new Rect(0,0,1180,720)); panel.UpdateLayout();
        Assert(buttons.Any(b=>Equals(b.Content,"紧急停止") && b.ActualHeight>=44),"stop remains allocated at tablet size");
        if(args.Length>0) {
            var small=new RenderTargetBitmap(1180,720,96,96,PixelFormats.Pbgra32);small.Render(panel);
            var png=new PngBitmapEncoder();png.Frames.Add(BitmapFrame.Create(small));
            using var stream=File.Create(Path.ChangeExtension(args[0],"tablet.png"));png.Save(stream);
        }
        var settingsWindow=new ConnectionSettingsWindow(Path.Combine(AppContext.BaseDirectory,"robotlink.json"));
        var settingsPanel=(FrameworkElement)settingsWindow.Content;
        settingsPanel.Measure(new Size(680,720));settingsPanel.Arrange(new Rect(0,0,680,720));settingsPanel.UpdateLayout();
        Assert(Descendants(settingsPanel).OfType<TextBox>().Count()==10,"main and backup connection fields present");
        if(args.Length>0) {
            var img=new RenderTargetBitmap(680,720,96,96,PixelFormats.Pbgra32);img.Render(settingsPanel);
            var png=new PngBitmapEncoder();png.Frames.Add(BitmapFrame.Create(img));
            using var stream=File.Create(Path.ChangeExtension(args[0],"settings.png"));png.Save(stream);
        }
        vm.Dispose();
        Console.WriteLine("PASS: offline WPF layout and input cancellation checks");
    }
    static IEnumerable<DependencyObject> Descendants(DependencyObject root)
    {
        for(int i=0;i<VisualTreeHelper.GetChildrenCount(root);i++)
        {
            var child=VisualTreeHelper.GetChild(root,i); yield return child;
            foreach(var descendant in Descendants(child)) yield return descendant;
        }
    }
    static void Assert(bool ok,string message) { if(!ok) throw new Exception(message); Console.WriteLine("PASS: "+message); }
}


