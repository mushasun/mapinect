using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ZhangCalibration
{
	public class NotifyablePoint : NotifyPropertyChanged
	{

		public const string PointProperty = "MyPoint";
		private System.Windows.Point _Point;
		public System.Windows.Point Point {
			get
			{
				return _Point;
			}
			set
			{
				_Point = value;
				FirePropertyChanged(PointProperty);
			}
		}

	}
}
