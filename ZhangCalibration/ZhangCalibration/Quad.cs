using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;
using System.Windows.Media;

namespace ZhangCalibration
{
	public class Quad : NotifyPropertyChanged.NotifyPropertyChanged
	{

		public Quad()
		{
			Points = new PointCollection();
		}

		public override string ToString()
		{
			return string.Join(", ", Points.Select(p => "{ " + p.ToString() + " }"));
		}

		public const string PointsProperty = "Points";
		private PointCollection myPoints;
		public PointCollection Points {
			get
			{
				return myPoints;
			}
			set
			{
				myPoints = value;
				FirePropertyChanged(PointsProperty);
			}
		}

	}
}
