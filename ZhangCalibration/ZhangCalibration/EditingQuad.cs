using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Data;

namespace ZhangCalibration
{
	public class EditingQuad : NotifyPropertyChanged
	{

		public EditingQuad(Quad quad)
		{
			this.Quad = quad;
			this.Polygon = new System.Windows.Shapes.Polygon();
			Polygon.Stroke = System.Windows.Media.Brushes.Red;
			Polygon.StrokeThickness = 1;
			Binding binding = new Binding();
			binding.Source = Quad;
			binding.Path = new System.Windows.PropertyPath("Points");
			Polygon.SetBinding(System.Windows.Shapes.Polygon.PointsProperty, binding);
		}

		~EditingQuad()
		{
		}

		public System.Windows.Shapes.Polygon Polygon { get; private set; }

		private Quad _Quad;
		public Quad Quad
		{
			get
			{
				return _Quad;
			}
			set
			{
				_Quad = value;
				FirePropertyChanged("Quad");
			}
		}

		private int _EditingPointIndex;
		public int EditingPointIndex
		{
			get
			{
				return _EditingPointIndex;
			}
			set
			{
				_EditingPointIndex = value;
				FirePropertyChanged("EditingPointIndex");
			}
		}

	}
}
