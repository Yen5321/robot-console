namespace RustRemoval.RobotConsole.ViewModels;

public sealed class InterlockItemViewModel(string name, bool isSatisfied) : ObservableObject
{
    private bool _isSatisfied = isSatisfied;
    public string Name { get; } = name;
    public bool IsSatisfied { get => _isSatisfied; set { if (Set(ref _isSatisfied, value)) Raise(nameof(StatusText)); } }
    public string StatusText => IsSatisfied ? "满足" : "不满足";
}
