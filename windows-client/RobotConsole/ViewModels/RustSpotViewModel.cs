using RustRemoval.RobotConsole.Models;

namespace RustRemoval.RobotConsole.ViewModels;

public sealed class RustSpotViewModel(RustSpot spot) : ObservableObject
{
    private bool _isSelected;
    public int Id => spot.Id;
    public double Left => spot.BoxX * 1000;
    public double Top => spot.BoxY * 562.5;
    public double Width => spot.BoxW * 1000;
    public double Height => spot.BoxH * 562.5;
    public string ConfidenceText => $"锈斑 {spot.Id}  {spot.Confidence:P0}";
    public bool IsSelected { get => _isSelected; set => Set(ref _isSelected, value); }
}
