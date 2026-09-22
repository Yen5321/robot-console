using System.Text.Json;
using System.IO;

namespace RustRemoval.RobotConsole.Services;

public sealed class RobotLinkSettings
{
    public string VideoTransport {get;set;}="WebRTC";
    public string WebRtcUrl {get;set;}="http://10.126.122.113:8889/robot/";
    public string? BackupRobotHost {get;set;}
    public string? BackupHttpUrl {get;set;}
    public string? BackupWebRtcUrl {get;set;}
    public string? LocalBindAddress {get;set;}
    public string? BackupLocalBindAddress {get;set;}
    public string Mode { get; set; } = "Mock";
    public string RobotHost { get; set; } = "192.168.1.50";
    public int UdpPort { get; set; } = 9000;
    public string MjpegUrl { get; set; } = "http://192.168.1.50:8080/stream.mjpg";
    public int CommandIntervalMs { get; set; } = 50;
    public int ControlTimeoutMs { get; set; } = 350;
    public int VideoTimeoutMs { get; set; } = 2500;

    public static RobotLinkSettings Load(string path)
    {
        if (!File.Exists(path)) return new RobotLinkSettings();
        var json = File.ReadAllText(path);
        return JsonSerializer.Deserialize<RobotLinkSettings>(json, new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true,
            ReadCommentHandling = JsonCommentHandling.Skip,
            AllowTrailingCommas = true,
        }) ?? new RobotLinkSettings();
    }

    public void Validate()
    {
        if (!Mode.Equals("Mock", StringComparison.OrdinalIgnoreCase) &&
            !Mode.Equals("Real", StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("robotlink.json Mode must be Mock or Real");
        if(VideoTransport is not ("WebRTC" or "MJPEG")) throw new InvalidOperationException("VideoTransport must be WebRTC or MJPEG");
        if(VideoTransport=="WebRTC" && (!Uri.TryCreate(WebRtcUrl,UriKind.Absolute,out var video) || video.Scheme is not ("http" or "https"))) throw new InvalidOperationException("Invalid WebRtcUrl");
        if(!string.IsNullOrWhiteSpace(BackupRobotHost) && (string.IsNullOrWhiteSpace(BackupHttpUrl)||string.IsNullOrWhiteSpace(BackupWebRtcUrl))) throw new InvalidOperationException("Backup path needs host, HTTP and WebRTC URLs");
        if (UdpPort is < 1 or > 65535) throw new InvalidOperationException("UdpPort must be 1..65535");
        if (Uri.CheckHostName(RobotHost)==UriHostNameType.Unknown) throw new InvalidOperationException("机器人地址无效，请填写 IP 或主机名，不包含端口");
        foreach(var address in new[]{LocalBindAddress,BackupLocalBindAddress})
            if(!string.IsNullOrWhiteSpace(address) && !System.Net.IPAddress.TryParse(address,out _)) throw new InvalidOperationException("网卡绑定地址必须是本机 IP");
        if(!string.IsNullOrWhiteSpace(BackupRobotHost)) {
            if(Uri.CheckHostName(BackupRobotHost)==UriHostNameType.Unknown) throw new InvalidOperationException("备用机器人地址无效");
            foreach(var address in new[]{BackupHttpUrl,BackupWebRtcUrl})
                if(!Uri.TryCreate(address,UriKind.Absolute,out var backupUri)||backupUri.Scheme is not ("http" or "https")) throw new InvalidOperationException("备用地址必须是完整 HTTP(S) 地址");
        }
        if (CommandIntervalMs is < 50 or > 100) throw new InvalidOperationException("CommandIntervalMs must be 50..100");
        if (ControlTimeoutMs < 250) throw new InvalidOperationException("ControlTimeoutMs must be at least 250");
        if (VideoTimeoutMs < 500) throw new InvalidOperationException("VideoTimeoutMs must be at least 500");
        if (!Uri.TryCreate(MjpegUrl, UriKind.Absolute, out var uri) || uri.Scheme != Uri.UriSchemeHttp)
            throw new InvalidOperationException("MjpegUrl must be an absolute http URL");
    }

    public void Save(string path)
    {
        Validate();
        var temp=path+".tmp";
        File.WriteAllText(temp,JsonSerializer.Serialize(this,new JsonSerializerOptions { WriteIndented=true }));
        if(File.Exists(path)) File.Copy(path,path+".bak",true);
        File.Move(temp,path,true);
    }
}
