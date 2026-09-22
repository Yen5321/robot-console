namespace RustRemoval.RobotConsole.ViewModels;

public sealed class JointMarginViewModel(string name) : ObservableObject
{
    private float _value;
    public string Name { get; } = name;
    public float Value { get => _value; set { if (Set(ref _value, value)) Raise(nameof(IsWarning)); } }
    public bool IsWarning => Value < 25;
}
