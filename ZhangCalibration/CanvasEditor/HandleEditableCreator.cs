using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Shapes;
using System.Windows.Data;
using PropertyChangedLibrary;

namespace CanvasEditor
{
	class HandleEditableCreator
	{

		internal static UIElement CreateHandle(PointNotifyPropertyChanged point)
		{
			Rectangle handle = new Rectangle();

			handle.SetValue(Canvas.WidthProperty, HandleEditableConverter.HandleSize.Width);
			handle.SetValue(Canvas.HeightProperty, HandleEditableConverter.HandleSize.Height);

			Binding xBinding = new Binding(PointNotifyPropertyChanged.XProperty);
			xBinding.Source = point;
			xBinding.Converter = new HandleEditableConverter();
			handle.SetBinding(Canvas.LeftProperty, xBinding);

			Binding yBinding = new Binding(PointNotifyPropertyChanged.YProperty);
			yBinding.Source = point;
			yBinding.Converter = new HandleEditableConverter();
			handle.SetBinding(Canvas.TopProperty, yBinding);

			return handle;
		}

	}

	[ValueConversion(typeof(double), typeof(double))]
	class HandleEditableConverter : IValueConverter
	{

		static public readonly Size HandleSize = new Size(6, 6);

		public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value - HandleSize.Width * .5;
		}

		public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value + HandleSize.Width * .5;
		}
	}
}
