using System.ComponentModel;
using System.Windows.Controls;
using System.IO;
using System.Windows;
using System.Windows.Input;
using RustRemoval.RobotConsole.Controls;
using RustRemoval.RobotConsole.Services;
using RustRemoval.RobotConsole.ViewModels;

namespace RustRemoval.RobotConsole.Views;

public partial class MainWindow : Window
{
    private readonly MainViewModel _viewModel;

    public MainWindow() : this(null) { }

    public MainWindow(MainViewModel? suppliedViewModel)
    {
        InitializeComponent();
        var settingsPath = Path.Combine(AppContext.BaseDirectory, "robotlink.json");
        var directionPath = Path.Combine(AppContext.BaseDirectory, "jog-directions.json");
        _viewModel = suppliedViewModel ?? new MainViewModel(RobotLinkFactory.Create(settingsPath), JogDirections.Load(directionPath), directionPath);
        DataContext = _viewModel;
        WebRtcVideo.FramePresented += _viewModel.VideoPresented;
        ArmJoystick.CanBeginInteraction = () => IsActive && _axisButton is null && _viewModel.BeginArmInput("joystick");
        ArmJoystick.InteractionEnded += () => _viewModel.StopArmJog();
        _viewModel.ArmInputCancelled += CancelArmCaptures;
        Deactivated += (_, _) => _viewModel.StopArmJog();
    }

    private async void Window_Loaded(object sender, RoutedEventArgs e) => await _viewModel.InitializeAsync();

    private void Window_Closing(object? sender, CancelEventArgs e) { WebRtcVideo.Close(); _viewModel.Dispose(); }

    private async void ConnectionSettings_Click(object sender,RoutedEventArgs e)
    {
        var button=(Button)sender;button.IsEnabled=false;
        try {
            await _viewModel.DisconnectForSettingsAsync();
            WebRtcVideo.Close();
            var dialog=new ConnectionSettingsWindow(Path.Combine(AppContext.BaseDirectory,"robotlink.json")){Owner=this};
            dialog.ShowDialog();
            // Recreate the disconnected video/link components even on Cancel.
            // Reconnection only sends neutral input; recovery remains explicit.
            var next=new MainWindow();Application.Current.MainWindow=next;next.Show();Close();
        } catch(Exception ex) { MessageBox.Show(this,ex.Message,"连接设置未完成"); }
        finally {button.IsEnabled=true;}
    }

    private void Diagnostics_Click(object sender,RoutedEventArgs e)
    {
        _viewModel.StopArmJog();
        new DiagnosticsWindow(_viewModel){Owner=this}.Show();
    }

    private void Window_KeyDown(object sender, KeyEventArgs e)
    {
        if (e.Key == Key.D && Keyboard.Modifiers == (ModifierKeys.Control | ModifierKeys.Shift))
        {
            DebugPanel.Visibility = DebugPanel.Visibility == Visibility.Visible ? Visibility.Collapsed : Visibility.Visible;
            e.Handled = true;
        }
    }

    private Button? _axisButton;
    private TouchDevice? _axisTouch;
    private void CancelArmCaptures()
    {
        var button = _axisButton; _axisButton = null;
        var touch = _axisTouch; _axisTouch = null;
        button?.ReleaseMouseCapture();
        if (touch is not null) button?.ReleaseTouchCapture(touch);
        ArmJoystick.CancelInput();
    }
    private void Axis_MouseDown(object sender, MouseButtonEventArgs e)
    {
        if (_axisButton is not null) { e.Handled = true; return; }
        var button = (Button)sender;
        if (!IsActive || !button.IsEnabled || !_viewModel.BeginArmInput("axis")) return;
        if (!button.CaptureMouse()) { _viewModel.StopArmJog(); return; }
        _axisButton = button;
        _viewModel.UpdateArmAxis((string)button.Tag);
        e.Handled = true;
    }
    private void Axis_MouseUp(object sender, MouseButtonEventArgs e)
    {
        if (ReferenceEquals(sender, _axisButton)) { _viewModel.StopArmJog(); _axisButton = null; ((Button)sender).ReleaseMouseCapture(); }
        e.Handled = true;
    }
    private void Axis_LostMouseCapture(object sender, MouseEventArgs e)
    {
        if (ReferenceEquals(sender, _axisButton)) { _axisButton = null; _viewModel.StopArmJog(); }
    }
    private void Axis_TouchDown(object sender, TouchEventArgs e)
    {
        e.Handled = true;
        if (_axisButton is not null) return;
        var button = (Button)sender;
        if (!IsActive || !button.IsEnabled || !_viewModel.BeginArmInput("axis")) return;
        if (!button.CaptureTouch(e.TouchDevice)) { _viewModel.StopArmJog(); return; }
        _axisTouch = e.TouchDevice;
        _axisButton = button;
        _viewModel.UpdateArmAxis((string)button.Tag);
    }
    private void Axis_TouchUp(object sender, TouchEventArgs e)
    {
        if (ReferenceEquals(sender, _axisButton) && _axisTouch == e.TouchDevice) _viewModel.StopArmJog();
        e.Handled = true;
    }
    private void Axis_LostTouchCapture(object sender, TouchEventArgs e)
    {
        if (ReferenceEquals(sender, _axisButton) && _axisTouch == e.TouchDevice) _viewModel.StopArmJog();
    }

    private void DriveJoystick_ValueChanged(object? sender, JoystickChangedEventArgs e) => _viewModel.UpdateDriveJoystick(e.X, e.Y);
    private void ArmJoystick_ValueChanged(object? sender, JoystickChangedEventArgs e) => _viewModel.UpdateArmJoystick(e.X, e.Y);
}

