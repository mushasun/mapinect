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
			this.Quad = quad;
			this.Polygon = new Polygon();
			Polygon.Fill = PolygonFillColor;
			Polygon.StrokeThickness = 1;
			Polygon.Points = Quad.Points;
		}

		~EditingQuad()
		{
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

	}

}
