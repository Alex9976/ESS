using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using HarfBuzzSharp;
using LiveChartsCore;
using LiveChartsCore.Defaults;
using LiveChartsCore.SkiaSharpView;

namespace StmReader
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            Task.Run(() => SerialIO.Start());
            Task.Run(() =>
            {
                while (true)
                {
                    //int reg = -1;
                    //int lig = -1;
                    //int temp = -1;
                    //int tempDHT = -1;
                    //int hum = -1;

                    //if (SerialIO.ResistorBuffer.Count > 0)
                    //{
                    //    reg = SerialIO.ResistorBuffer[0];
                    //    SerialIO.ResistorBuffer.RemoveAt(0);
                    //}
                    //if (SerialIO.LightBuffer.Count > 0)
                    //{
                    //    lig = SerialIO.LightBuffer[0];
                    //    SerialIO.LightBuffer.RemoveAt(0);
                    //}
                    //if (SerialIO.TempBuffer.Count > 0)
                    //{
                    //    temp = SerialIO.TempBuffer[0];
                    //    SerialIO.TempBuffer.RemoveAt(0);
                    //}
                    //if (SerialIO.TempDHTBuffer.Count > 0)
                    //{
                    //    tempDHT = SerialIO.TempDHTBuffer[0];
                    //    SerialIO.TempDHTBuffer.RemoveAt(0);
                    //}
                    //if (SerialIO.HumidityBuffer.Count > 0)
                    //{
                    //    hum = SerialIO.HumidityBuffer[0];
                    //    SerialIO.HumidityBuffer.RemoveAt(0);
                    //}

                    //Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ThreadStart)delegate ()
                    //{
                    //    if (reg != -1)
                    //    {
                    //        SerialIO.ResistorValues.Add(new ObservableValue(reg));
                    //        if (SerialIO.ResistorValues.Count > 500)
                    //        {
                    //            SerialIO.ResistorValues.RemoveAt(0);
                    //        }
                    //    }
                    //    if (lig != -1)
                    //    {
                    //        SerialIO.LightValues.Add(new ObservableValue(lig));
                    //        if (SerialIO.LightValues.Count > 500)
                    //        {
                    //            SerialIO.LightValues.RemoveAt(0);
                    //        }
                    //    }
                    //    if (temp != -1)
                    //    {
                    //        SerialIO.TempValues.Add(new ObservableValue(temp));
                    //        if (SerialIO.TempValues.Count > 500)
                    //        {
                    //            SerialIO.TempValues.RemoveAt(0);
                    //        }
                    //    }
                    //    if (tempDHT != -1)
                    //    {
                    //        SerialIO.TempDHTValues.Add(new ObservableValue(tempDHT));
                    //        if (SerialIO.TempDHTValues.Count > 500)
                    //        {
                    //            SerialIO.TempDHTValues.RemoveAt(0);
                    //        }
                    //    }
                    //    if (hum != -1)
                    //    {
                    //        SerialIO.HumidityValues.Add(new ObservableValue(hum));
                    //        if (SerialIO.HumidityValues.Count > 500)
                    //        {
                    //            SerialIO.HumidityValues.RemoveAt(0);
                    //        }
                    //    }
                    //});



                    if (SerialIO.ResistorBuffer.Count > 0)
                    {
                        var val = SerialIO.ResistorBuffer[0];
                        Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ThreadStart)delegate ()
                        {
                            SerialIO.ResistorValues.Add(new ObservableValue(val));
                            if (SerialIO.ResistorValues.Count > 500)
                            {
                                SerialIO.ResistorValues.RemoveAt(0);
                            }
                        });
                        SerialIO.ResistorBuffer.RemoveAt(0);
                    }
                    if (SerialIO.LightBuffer.Count > 0)
                    {
                        var val = SerialIO.LightBuffer[0];
                        Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ThreadStart)delegate ()
                        {
                            SerialIO.LightValues.Add(new ObservableValue(val));
                            if (SerialIO.LightValues.Count > 500)
                            {
                                SerialIO.LightValues.RemoveAt(0);
                            }
                        });
                        SerialIO.LightBuffer.RemoveAt(0);
                    }
                    if (SerialIO.TempBuffer.Count > 0)
                    {
                        var val = SerialIO.TempBuffer[0];
                        Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ThreadStart)delegate ()
                        {
                            SerialIO.TempValues.Add(new ObservableValue(val));
                            if (SerialIO.TempValues.Count > 500)
                            {
                                SerialIO.TempValues.RemoveAt(0);
                            }
                        });
                        SerialIO.TempBuffer.RemoveAt(0);
                    }
                    if (SerialIO.TempDHTBuffer.Count > 0)
                    {
                        var val = SerialIO.TempDHTBuffer[0];
                        Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ThreadStart)delegate ()
                        {
                            SerialIO.TempDHTValues.Add(new ObservableValue(val));
                            if (SerialIO.TempDHTValues.Count > 500)
                            {
                                SerialIO.TempDHTValues.RemoveAt(0);
                            }
                        });
                        SerialIO.TempDHTBuffer.RemoveAt(0);
                    }
                    if (SerialIO.HumidityBuffer.Count > 0)
                    {
                        var val = SerialIO.HumidityBuffer[0];
                        Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ThreadStart)delegate ()
                        {
                            SerialIO.HumidityValues.Add(new ObservableValue(val));
                            if (SerialIO.HumidityValues.Count > 500)
                            {
                                SerialIO.HumidityValues.RemoveAt(0);
                            }
                        });
                        SerialIO.HumidityBuffer.RemoveAt(0);
                    }
                }
            });
        }

        private void togRGBR_Click(object sender, RoutedEventArgs e)
        {
            if (togRGBR.IsChecked == false)
            {
                SerialIO.Send("RGB_R_OFF");
            }
            else
            {
                SerialIO.Send("RGB_R_ON");
            }
        }

        private void togRGBG_Click(object sender, RoutedEventArgs e)
        {
            if (togRGBG.IsChecked == false)
            {
                SerialIO.Send("RGB_G_OFF");
            }
            else
            {
                SerialIO.Send("RGB_G_ON");
            }
        }

        private void togRGBB_Click(object sender, RoutedEventArgs e)
        {
            if (togRGBB.IsChecked == false)
            {
                SerialIO.Send("RGB_B_OFF");
            }
            else
            {
                SerialIO.Send("RGB_B_ON");
            }
        }

        private void togLEDR_Click(object sender, RoutedEventArgs e)
        {
            if (togLEDR.IsChecked == false)
            {
                SerialIO.Send("LED_R_OFF");
            }
            else
            {
                SerialIO.Send("LED_R_ON");
            }
        }

        private void togLEDB_Click(object sender, RoutedEventArgs e)
        {
            if (togLEDB.IsChecked == false)
            {
                SerialIO.Send("LED_B_OFF");
            }
            else
            {
                SerialIO.Send("LED_B_ON");
            }
        }
    }
}
