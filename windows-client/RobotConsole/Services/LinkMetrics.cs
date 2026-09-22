namespace RustRemoval.RobotConsole.Services;

public record NetworkMetrics(double? RttMs, double? AckTimeoutPercent, int Completed, int TimedOut);

/// <summary>Client-clock RTT and rolling ACK timeouts; not fabricated one-way latency.</summary>
public sealed class LinkMetrics
{
    private sealed record Ticket(ushort Seq, uint Stamp, long SentAt) { public long? AckAt; }
    private readonly List<Ticket> _tickets = [];
    private readonly object _lock = new();
    private double? _rtt;
    public void Sent(ushort seq, uint stamp, long now)
    {
        lock (_lock) { Trim(now); _tickets.Add(new(seq,stamp,now)); }
    }
    public bool Acknowledge(ushort seq, uint stamp, long now)
    {
        lock (_lock)
        {
            var ticket = _tickets.LastOrDefault(t => t.Seq==seq && t.Stamp==stamp);
            if (ticket is null || ticket.AckAt is not null || now-ticket.SentAt>=1000) return false;
            ticket.AckAt=now; _rtt=now-ticket.SentAt; return true;
        }
    }
    public NetworkMetrics Snapshot(long now)
    {
        lock (_lock)
        {
            Trim(now);
            // Compare an equal-age cohort; do not bias loss low by counting early ACKs only.
            var completed=_tickets.Where(t=>now-t.SentAt>=1000).ToArray();
            int lost=completed.Count(t=>t.AckAt is null);
            var newestAck=_tickets.Where(t=>t.AckAt is not null).Select(t=>t.AckAt!.Value).DefaultIfEmpty(long.MinValue/2).Max();
            return new(now-newestAck<=2000 ? _rtt : null,
                completed.Length==0 ? null : 100.0*lost/completed.Length,completed.Length,lost);
        }
    }
    public void Reset() { lock (_lock) { _tickets.Clear(); _rtt=null; } }
    private void Trim(long now) => _tickets.RemoveAll(t=>now-t.SentAt>6000);
}
