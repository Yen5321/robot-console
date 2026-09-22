using System.Buffers.Binary;
namespace RustRemoval.RobotConsole.Services;

public record SensorTelemetry(uint ValidMask, float[] Pose, float[] MotorCurrentA, float BatteryVoltageV, float MagneticForceN);
public static class WireV6
{
    public static ushort Crc(ReadOnlySpan<byte> data)
    {
        ushort crc=0xffff;
        foreach(byte b in data) { crc^=(ushort)(b<<8); for(int i=0;i<8;i++) crc=(ushort)((crc&0x8000)!=0 ? (crc<<1)^0x1021 : crc<<1); }
        return crc;
    }
    public static bool Valid(ReadOnlySpan<byte> d,int length)=>d.Length==length && Crc(d[..^2])==BinaryPrimitives.ReadUInt16LittleEndian(d[^2..]);
    public static byte[] Control(uint session,ushort seq,uint stamp,bool uiAlive,RobotControlState state)
    {
        var old=RobotWireProtocol.PackControl(seq,stamp,0,state);
        var d=new byte[32]; var s=d.AsSpan();
        BinaryPrimitives.WriteUInt32LittleEndian(s,session);
        old.AsSpan(0,6).CopyTo(s[4..10]);
        d[10]=(byte)((byte)state.Mode|(state.Estop?4:0)|(state.LaserEnable?8:0)|(uiAlive?16:0));
        old.AsSpan(7,16).CopyTo(s[11..27]); d[27]=old[24]; old.AsSpan(26,2).CopyTo(s[28..30]);
        BinaryPrimitives.WriteUInt16LittleEndian(s[30..],Crc(s[..30])); return d;
    }
    public static bool Ack(ReadOnlySpan<byte> d,uint session,out ushort seq,out uint stamp)
    {
        seq=0;stamp=0;
        if(!Valid(d,17)||!d[..4].SequenceEqual("ACK6"u8)||BinaryPrimitives.ReadUInt32LittleEndian(d[4..])!=session) return false;
        seq=BinaryPrimitives.ReadUInt16LittleEndian(d[8..]);stamp=BinaryPrimitives.ReadUInt32LittleEndian(d[10..]);return true;
    }
    public static bool Telemetry(ReadOnlySpan<byte> d,uint session,out RobotWireTelemetry wire,out uint stamp,out SensorTelemetry? sensors)
    {
        wire=default;stamp=0;sensors=null;
        if(!Valid(d,97)||!d[..4].SequenceEqual("TLM6"u8)||BinaryPrimitives.ReadUInt32LittleEndian(d[4..])!=session) return false;
        if(!RobotWireProtocol.TryParseTelemetry(d.Slice(18,21),out wire)) return false;
        if(wire.Sequence!=BinaryPrimitives.ReadUInt16LittleEndian(d[8..])) return false;
        stamp=BinaryPrimitives.ReadUInt32LittleEndian(d[10..]);
        uint mask=BinaryPrimitives.ReadUInt32LittleEndian(d[14..]);
        var values=new float[14];for(int i=0;i<14;i++) values[i]=BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(d[(39+4*i)..]));
        sensors=new(mask,values[..6],values[6..12],values[12],values[13]);return true;
    }
    public static bool Newer(ushort next,ushort last)=>unchecked((ushort)(next-last)) is >0 and <0x8000;
    public static int Age(uint now,uint stamp)=>unchecked((int)(now-stamp));
}
