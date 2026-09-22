using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
namespace RustRemoval.RobotConsole.Controls;
public sealed class JoystickChangedEventArgs(double x, double y) : EventArgs
{
    public double X { get; } = x;
    public double Y { get; } = y;
}
public partial class VirtualJoystick : UserControl
{
    private bool _dragging;
    private TouchDevice? _touch;
    public Func<bool>? CanBeginInteraction { get; set; }
    public event EventHandler<JoystickChangedEventArgs>? ValueChanged;
    public event Action? InteractionEnded;
    public VirtualJoystick()
    {
        InitializeComponent();
        IsEnabledChanged += (_, _) => { if (!IsEnabled) CancelInput(); };
    }
    private void Update(Point point)
    {
        var radius = Math.Max(1, Math.Min(ActualWidth, ActualHeight)/2 - 40);
        var dx = point.X - ActualWidth/2;
        var dy = point.Y - ActualHeight/2;
        var distance = Math.Sqrt(dx*dx+dy*dy);
        if (distance > radius) { dx *= radius/distance; dy *= radius/distance; }
        KnobTransform.X = dx; KnobTransform.Y = dy;
        ValueChanged?.Invoke(this, new(dx/radius, -dy/radius));
    }
    public void CancelInput()
    {
        var wasDragging = _dragging;
        _dragging = false;
        var touch = _touch; _touch = null;
        KnobTransform.X = KnobTransform.Y = 0;
        Surface.ReleaseMouseCapture();
        if (touch is not null) Surface.ReleaseTouchCapture(touch);
        if (wasDragging) { ValueChanged?.Invoke(this, new(0,0)); InteractionEnded?.Invoke(); }
    }
    private void Surface_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        e.Handled = true;
        if (_dragging || !IsEnabled) return;
        if (CanBeginInteraction is not null && !CanBeginInteraction()) return;
        if (!Surface.CaptureMouse()) { InteractionEnded?.Invoke(); return; }
        _dragging = true; Update(e.GetPosition(this));
    }
    private void Surface_MouseMove(object sender, MouseEventArgs e)
    {
        if (!_dragging || _touch is not null) return;
        if (e.LeftButton != MouseButtonState.Pressed) { CancelInput(); return; }
        Update(e.GetPosition(this));
    }
    private void Surface_MouseLeftButtonUp(object sender, MouseButtonEventArgs e) { if (_touch is null) CancelInput(); e.Handled = true; }
    private void Surface_LostMouseCapture(object sender, MouseEventArgs e) { if (_dragging && _touch is null) CancelInput(); }
    private void Surface_TouchDown(object sender, TouchEventArgs e)
    {
        e.Handled = true;
        if (_dragging || !IsEnabled) return;
        if (CanBeginInteraction is not null && !CanBeginInteraction()) return;
        if (!Surface.CaptureTouch(e.TouchDevice)) { InteractionEnded?.Invoke(); return; }
        _touch = e.TouchDevice; _dragging = true; Update(e.GetTouchPoint(this).Position);
    }
    private void Surface_TouchMove(object sender, TouchEventArgs e) { if (_dragging && _touch == e.TouchDevice) Update(e.GetTouchPoint(this).Position); e.Handled = true; }
    private void Surface_TouchUp(object sender, TouchEventArgs e) { if (_touch == e.TouchDevice) CancelInput(); e.Handled = true; }
    private void Surface_LostTouchCapture(object sender, TouchEventArgs e) { if (_touch == e.TouchDevice) CancelInput(); }
}
