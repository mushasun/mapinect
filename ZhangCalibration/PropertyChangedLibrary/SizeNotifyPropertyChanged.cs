using System.Windows;

namespace PropertyChangedLibrary
{
	public class SizeNotifyPropertyChanged : NotifyPropertyChanged.NotifyPropertyChanged
	{

		public SizeNotifyPropertyChanged(Size size)
		{
			Width = size.Width;
			Height = size.Height;
		}

		public Size Size
		{
			get
			{
				return new Size(Width, Height);
			}
		}

		public const string WidthProperty = "Width";
		private double myWidth;
		public double Width
		{
			get
			{
				return myWidth;
			}
			set
			{
				myWidth = value;
				FirePropertyChanged(WidthProperty);
			}
		}

		public const string HeightProperty = "Height";
		private double myHeight;
		public double Height
		{
			get
			{
				return myHeight;
			}
			set
			{
				myHeight = value;
				FirePropertyChanged(HeightProperty);
			}
		}

	}
}
