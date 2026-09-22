namespace RustRemoval.RobotConsole.Services;

public static class RobotLinkFactory
{
    public static IRobotLink Create(string settingsPath)
    {
        var settings = RobotLinkSettings.Load(settingsPath);
        settings.Validate();
        return settings.Mode.Equals("Real", StringComparison.OrdinalIgnoreCase)
            ? new RealRobotLink(settings)
            : new MockRobotLink();
    }
}
