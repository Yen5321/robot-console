using System.Windows;
using System.Diagnostics;
using System.Threading;

namespace RustRemoval.RobotConsole;

public partial class App : Application
{
    private Mutex? _instanceMutex;
    protected override void OnStartup(StartupEventArgs e)
    {
        _instanceMutex = new Mutex(true, @"Local\RobotConsoleGroundJogV6", out var created);
        var current = Environment.ProcessId;
        var other = Process.GetProcessesByName(Process.GetCurrentProcess().ProcessName).FirstOrDefault(p => p.Id != current);
        if (!created || other is not null)
        {
            MessageBox.Show("已有机械臂控制程序运行。请先退出旧窗口，再启动 v8.1，避免多端同时发送控制包。", "控制程序重复运行");
            Shutdown();
            return;
        }
        base.OnStartup(e);
    }
    protected override void OnExit(ExitEventArgs e)
    {
        _instanceMutex?.Dispose();
        base.OnExit(e);
    }
}

