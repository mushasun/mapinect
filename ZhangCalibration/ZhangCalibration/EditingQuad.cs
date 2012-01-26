using System.Windows.Shapes;
using System.Windows.Media;

namespace ZhangCalibration
{
	public class EditingQuad : NotifyPropertyChanged.NotifyPropertyChanged
	{

		private Brush UnselectedStrokeColor = Brushes.Gray;
		private Brush SelectedStrokeColor = Brushes.Blue;
		private Brush PolygonFillColor = new SolidColorBrush(Color.FromArgb(64, 255, 255, 0));
		private Brush HandleColor = new SolidColorBrush(Color.FromArgb(192, 255, 0, 0));

		public EditingQuad(Quad quad)
		{
			Quad = quad;
			Polygon = new Polygon();
			Polygon.Fill = PolygonFillColor;
			Polygon.StrokeThickness = 1;
			Polygon.Points = Quad.Points;
			Polygon.PreviewMouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(Polygon_PreviewMouseLeftButtonDown);
			Selected = false;
		}

		~EditingQuad()
		{
			Polygon.PreviewMouseLeftButtonDown -= Polygon_PreviewMouseLeftButtonDown;
			Quad = null;
			Polygon = null;
		}

		public override string ToString()
		{
			return Quad.ToString();
		}

		public const string QuadProperty = "Quad";
		private Quad myQuad;
		public Quad Quad
		{
			get
			{
				return myQuad;
			}
			set
			{
				myQuad = value;
				FirePropertyChanged(QuadProperty);
			}
		}

		public Polygon Polygon { get; private set; }

		private void Polygon_PreviewMouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (!e.Handled)
			{
				e.Handled = true;
				Selected = true;
			}
		}

		public const string SelectedProperty = "Selected";
		private bool mySelected;
		public bool Selected
		{
			get
			{
				return mySelected;
			}
			set
			{
				mySelected = value;
				FirePropertyChanged(SelectedProperty);
			}
		}

	}

}
