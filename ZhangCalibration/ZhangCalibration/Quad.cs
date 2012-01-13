using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ZhangCalibration
{
	public class Quad : NotifyPropertyChanged
	{

		public Quad()
		{
			Points = new List<System.Windows.Point>();
		}

		public override string ToString()
		{
			string result = "";
			Points.ForEach(p => result += "{ " + p.ToString() + " }");
			return result;
		}

		public const string PointsProperty = "Points";
		private List<System.Windows.Point> _Points;
		public List<System.Windows.Point> Points {
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
