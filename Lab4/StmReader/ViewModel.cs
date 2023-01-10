using System.Collections.ObjectModel;
using LiveChartsCore;
using LiveChartsCore.Defaults;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.Painting;
using SkiaSharp;

namespace StmReader;

public class ViewModel
{
    public ISeries[] Main { get; set; } = new ISeries[]
        {
                new LineSeries<ObservableValue>
                {
                    Values = SerialIO.ResistorValues,
                    Name = "Potentiometer",
                    Fill = null,
                    Stroke = new SolidColorPaint(SKColors.Red, 2),
                    GeometryStroke = new SolidColorPaint(SKColors.Red, 2),
                    GeometrySize = 0,
                    LineSmoothness = 0
                },
                new LineSeries<ObservableValue>
                {
                    Values = SerialIO.LightValues,
                    Name = "Lighting",
                    Fill = null,
                    Stroke = new SolidColorPaint(SKColors.Green, 2),
                    GeometryStroke = new SolidColorPaint(SKColors.Green, 2),
                    GeometrySize = 0,
                    LineSmoothness = 0
                }
        };

    public ISeries[] Temp { get; set; } = new ISeries[]
        {
                new LineSeries<ObservableValue>
                {
                    Values = SerialIO.TempValues,
                    Name = "Temperature",
                    Stroke = new SolidColorPaint(SKColors.Orange, 2),
                    GeometryStroke = new SolidColorPaint(SKColors.Orange, 2),
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0
                },
                new LineSeries<ObservableValue>
                {
                    Values = SerialIO.TempDHTValues,
                    Stroke = new SolidColorPaint(SKColors.Blue, 2),
                    GeometryStroke = new SolidColorPaint(SKColors.Blue, 2),
                    Name = "Temperature (DHT)",
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0
                }
        };

    public ISeries[] Hum { get; set; } = new ISeries[]
        {
                new LineSeries<ObservableValue>
                {
                    Values = SerialIO.HumidityValues,
                    Name = "Humidity",
                    Stroke = new SolidColorPaint(SKColors.Indigo, 2),
                    GeometryStroke = new SolidColorPaint(SKColors.Indigo, 2),
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0
                },
        };
}