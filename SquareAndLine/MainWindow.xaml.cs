using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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

namespace SquareAndLine
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        private bool isDragging;
        private Point box1Point;
        private Point box2Point;
        public MainWindow()
        {
            InitializeComponent();
        }

        private void box1_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            isDragging = true;
            box1Point = e.GetPosition(null);
            box1.CaptureMouse();
            UpdateConnector();
        }
        private void box2_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            isDragging = true;
            box2Point = e.GetPosition(null);
            box2.CaptureMouse();
            UpdateConnector();
        }
        private void box1_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                Point currentPoint = e.GetPosition(null);
                double deltaX = currentPoint.X - box1Point.X;
                double deltaY = currentPoint.Y - box1Point.Y;
                double newLeft = Canvas.GetLeft(box1) + deltaX;
                double newTop = Canvas.GetTop(box1) + deltaY;
                Canvas.SetLeft(box1, newLeft);
                Canvas.SetTop(box1, newTop);
                box1Point = currentPoint;
                UpdateConnector();
            }
        }
        private void box2_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                Point currentPoint = e.GetPosition(null);
                double deltaX = currentPoint.X - box2Point.X;
                double deltaY = currentPoint.Y - box2Point.Y;
                double newLeft = Canvas.GetLeft(box2) + deltaX;
                double newTop = Canvas.GetTop(box2) + deltaY;
                Canvas.SetLeft(box2, newLeft);
                Canvas.SetTop(box2, newTop);
                box2Point = currentPoint;
                UpdateConnector();
            }
        }
        private void box1_PreviewMouseUp(object sender, MouseButtonEventArgs e)
        {
            isDragging = false;
            box1.ReleaseMouseCapture();
        }
        private void box2_PreviewMouseUp(object sender, MouseButtonEventArgs e)
        {
            isDragging = false;
            box2.ReleaseMouseCapture();
        }
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            polygon.IsHitTestVisible = false;
            UpdateConnector();
        }
        private Point GetLeftEdgeCenter(UIElement element)
        {
            double left = Canvas.GetLeft(element);
            double top = Canvas.GetTop(element);
            double height = (element as FrameworkElement).ActualHeight;
            return new Point(left, top + height / 2);
        }

        private Point GetRightEdgeCenter(UIElement element)
        {
            double left = Canvas.GetLeft(element);
            double top = Canvas.GetTop(element);
            double width = (element as FrameworkElement).ActualWidth;
            double height = (element as FrameworkElement).ActualHeight;
            return new Point(left + width, top + height / 2);
        }

        private void UpdateConnector()
        {
            var start = GetRightEdgeCenter(box1);
            var end = GetLeftEdgeCenter(box2);
            var points = new List<Point> { start };

            points.AddRange(ExtendLineAroundPolygon(start, end, GetCornerPoints()));

            connector.Points = new PointCollection(points);
        }
        private List<Point> ExtendLineAroundPolygon(Point start, Point end, List<Point> polygonPoints)
        {
            var extendedPoints = new List<Point> { start };

            // 检查start到end的直线是否与多边形相交
            if (!IsLineIntersectingPolygon(polygonPoints, start, end))
            {
                extendedPoints.Add(end);
                return extendedPoints;
            }

            // 寻找一条绕过多边形的最短路径
            var shortestPath = FindShortestPathAroundPolygon(polygonPoints, start, end);

            // 添加路径上的点到extendedPoints
            extendedPoints.AddRange(shortestPath);
            extendedPoints.Add(end);

            return extendedPoints;
        }
        private List<Point> FindShortestPathAroundPolygon(List<Point> polygonPoints, Point start, Point end)
        {
            List<List<Point>> paths = new List<List<Point>>();
            for (int i = 0; i < polygonPoints.Count; i++)
            {
                var path = new List<Point> { start };
                int currentIndex = i;
                int previousIndex = (i - 1 + polygonPoints.Count) % polygonPoints.Count;
                while (IsLineIntersectingPolygon(polygonPoints, start, polygonPoints[currentIndex]) || IsLineIntersectingPolygon(polygonPoints, polygonPoints[currentIndex], end))
                {
                    path.Add(polygonPoints[currentIndex]);
                    currentIndex = (currentIndex + 1) % polygonPoints.Count;

                    if (currentIndex == previousIndex)
                    {
                        break;
                    }
                }
                path.Add(polygonPoints[currentIndex]);
                paths.Add(path);
            }

            // 寻找最短路径，同时考虑折叠次数
            double minPathScore = double.MaxValue;
            List<Point> shortestPath = null;
            foreach (var path in paths)
            {
                double pathLength = 0;
                int foldingCount = 0;
                for (int i = 0; i < path.Count - 1; i++)
                {
                    pathLength += (path[i] - path[i + 1]).Length;
                    if (i > 0)
                    {
                        Vector v1 = path[i] - path[i - 1];
                        Vector v2 = path[i + 1] - path[i];
                        if (!VectorsAreCollinear(v1, v2))
                        {
                            foldingCount++;
                        }
                    }
                }

                // 计算路径分数：路径长度和折叠次数的加权和
                double pathScore = pathLength + foldingCount * 50; // 50 是一个权重，可以根据需要调整
                if (pathScore < minPathScore)
                {
                    minPathScore = pathScore;
                    shortestPath = path;
                }
            }

            // 从最短路径中移除起始点
            shortestPath.RemoveAt(0);

            return shortestPath;
        }

        private bool VectorsAreCollinear(Vector v1, Vector v2)
        {
            const double tolerance = 1e-6;
            return Math.Abs(v1.X * v2.Y - v1.Y * v2.X) < tolerance;
        }

        private bool IsLineIntersectingPolygon(List<Point> polygonPoints, Point start, Point end)
        {
            return GeometryHelper.PolygonIntersectsLine(polygonPoints, start, end, out _);
        }
        private List<Point> GetCornerPoints()
        {
            List<Point> cornerPoints = new List<Point>
            {
                new Point(Canvas.GetLeft(polygon), Canvas.GetTop(polygon)),
                new Point(Canvas.GetLeft(polygon), Canvas.GetTop(polygon) + polygon.ActualHeight),
                new Point(Canvas.GetLeft(polygon) + polygon.ActualWidth, Canvas.GetTop(polygon) + polygon.ActualHeight),
                new Point(Canvas.GetLeft(polygon) + polygon.ActualWidth, Canvas.GetTop(polygon)),
            };
            return cornerPoints;
        }
    }

    public static class GeometryHelper
    {
        public static List<(Point, Point)> GetPolygonEdges(List<Point> polygonPoints)
        {
            return polygonPoints.Select((t, i) => (t, polygonPoints[(i + 1) % polygonPoints.Count])).ToList();
        }

        public static bool LineIntersectsLine(Point p1, Point p2, Point p3, Point p4, out Point intersection)
        {
            intersection = new Point();

            double s1_x, s1_y, s2_x, s2_y;
            s1_x = p2.X - p1.X;
            s1_y = p2.Y - p1.Y;
            s2_x = p4.X - p3.X;
            s2_y = p4.Y - p3.Y;

            double s, t;
            s = (-s1_y * (p1.X - p3.X) + s1_x * (p1.Y - p3.Y)) / (-s2_x * s1_y + s1_x * s2_y);
            t = (s2_x * (p1.Y - p3.Y) - s2_y * (p1.X - p3.X)) / (-s2_x * s1_y + s1_x * s2_y);

            if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
            {
                intersection.X = p1.X + (t * s1_x);
                intersection.Y = p1.Y + (t * s1_y);
                return true;
            }

            return false;
        }

        public static bool PolygonIntersectsLine(List<Point> polygonPoints, Point start, Point end, out Point intersection)
        {
            int pointCount = polygonPoints.Count;

            for (int i = 0; i < pointCount; i++)
            {
                Point a = polygonPoints[i];
                Point b = polygonPoints[(i + 1) % pointCount];

                if (LineIntersectsLine(start, end, a, b, out intersection))
                {
                    return true;
                }
            }

            intersection = new Point();
            return false;
        }

        public static Point RotateAndExtend(Point origin, Point end, double angle, double extensionLength)
        {
            Vector line = end - origin;

            double angleRadians = angle * Math.PI / 180.0;

            double rotatedX = line.X * Math.Cos(angleRadians) - line.Y * Math.Sin(angleRadians);
            double rotatedY = line.X * Math.Sin(angleRadians) + line.Y * Math.Cos(angleRadians);
            Vector rotatedLine = new Vector(rotatedX, rotatedY);

            rotatedLine.Normalize();
            Vector extension = rotatedLine * extensionLength;

            Vector extendedLine = rotatedLine + extension;

            Point extendedEndPoint = origin + extendedLine;
            return extendedEndPoint;
        }
    }

}
