using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using LiveChartsCore.Defaults;

namespace StmReader;

public static class SerialIO
{
    public static readonly ObservableCollection<ObservableValue> ResistorValues = new ObservableCollection<ObservableValue>();
    public static readonly ObservableCollection<ObservableValue> TempValues = new ObservableCollection<ObservableValue>();
    public static readonly ObservableCollection<ObservableValue> LightValues = new ObservableCollection<ObservableValue>();
    public static readonly ObservableCollection<ObservableValue> TempDHTValues = new ObservableCollection<ObservableValue>();
    public static readonly ObservableCollection<ObservableValue> HumidityValues = new ObservableCollection<ObservableValue>();
    public static List<int> ResistorBuffer = new List<int>();
    public static List<int> LightBuffer = new List<int>();
    public static List<double> TempBuffer = new List<double>();
    public static List<int> TempDHTBuffer = new List<int>();
    public static List<int> HumidityBuffer = new List<int>();

    static SerialPort Port = new SerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
    static bool Read = true;

    [STAThread]
    public static void Start()
    {
        SerialPortProgram();
    }

    public static void Send(string message)
    {
        Port.WriteLine(message);
    }
    private static void SerialPortProgram()
    {
        Console.WriteLine("Incoming Data:");
        Port.DataReceived += new SerialDataReceivedEventHandler(port_DataReceived);
        Port.Open();
    }

    private static void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        if (Read)
        {
            Task.Run(() =>
            {
                int readCount;
                int bufReady = 0;
                byte[] buf = new byte[128];
                while (bufReady < buf.Length)
                {
                    readCount = Port.Read(buf, bufReady, buf.Length - bufReady);
                    bufReady += readCount;
                }

                string data = Encoding.UTF8.GetString(buf);
                var values = data.Split("\n\r");
                foreach (var v in values)
                {
                    try
                    {
                        int resistorValue = 0;
                        int tempValue = 0;
                        int lightValue = 0;
                        int tempDHTValue = 0;
                        int humidityValue = 0;
                        string value = v.Trim(new char[] { '\r', '\n' });
                        if (value.Length == 26 && value.StartsWith("D:") && value.EndsWith(":D"))
                        {

                            resistorValue = int.Parse(value.Substring(2, 4));
                            lightValue = int.Parse(value.Substring(7, 4));
                            tempValue = int.Parse(value.Substring(12, 4));
                            tempDHTValue = int.Parse(value.Substring(17, 3));
                            humidityValue = int.Parse(value.Substring(21, 3));

                            if (resistorValue <= 4095)
                            {
                                ResistorBuffer.Add(resistorValue);
                            }
                            if (lightValue <= 4095)
                            {
                                LightBuffer.Add(lightValue);
                            }
                            if (tempValue <= 4095)
                            {
                                TempBuffer.Add(((double)tempValue / 4095.0) * 3.3 * 1000 / 10);
                            }
                            if (tempDHTValue <= 100)
                            {
                                TempDHTBuffer.Add(tempDHTValue);
                            }
                            if (humidityValue <= 100)
                            {
                                HumidityBuffer.Add(humidityValue);
                            }
                        }
                    }
                    catch
                    {
                    }
                }
            });
        }
    }
}
