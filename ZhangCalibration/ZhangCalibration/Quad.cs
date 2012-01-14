using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;
using System.Windows.Media;

namespace ZhangCalibration
{
	public class Quad : NotifyPropertyChanged
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
		private PointCollection _Points;
		public PointCollection Points {
			get
			{
				return _Points;
			}
			set
			{
				_Points = value;
				FirePropertyChanged(PointsProperty);
			}
		}

	}
}
