using System.Windows;

namespace PropertyChangedLibrary
{
	public class PointNotifyPropertyChanged : NotifyPropertyChanged.NotifyPropertyChanged
	{

		public PointNotifyPropertyChanged(Point point)
		{
			X = point.X;
			Y = point.Y;
		}

		public Point Point
		{
			get
			{
				return new Point(X, Y);
			}
		}

		public const string XProperty = "X";
		private double myX;
		public double X
		{
			get
			{
				return myX;
			}
			set
			{
				myX = value;
				FirePropertyChanged(XProperty);
			}
		}

		public const string YProperty = "Y";
		private double myY;
		public double Y
		{
			get
			{
				return myY;
			}
			set
			{
				myY = value;
				FirePropertyChanged(YProperty);
			}
		}

		public void Offset(double x, double y)
		{
			X += x;
			Y += y;
		}

	}
}
