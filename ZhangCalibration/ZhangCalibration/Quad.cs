using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;

namespace ZhangCalibration
{
	public class Quad : NotifyPropertyChanged
	{

		public Quad()
		{
			Points = new ObservableCollection<NotifyablePoint>();
		}

		public override string ToString()
		{
			return string.Join(", ", Points.Select(p => "{ " + p.ToString() + " }"));
		}

		public const string PointsProperty = "Points";
		private ObservableCollection<NotifyablePoint> _Points;
		public ObservableCollection<NotifyablePoint> Points {
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
