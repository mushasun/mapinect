using System;
using PropertyChangedLibrary;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;

namespace CanvasEditor
{
	internal class EditableAlteringObject
	{
		internal EditableAlteringObject(EditableAlteringManager manager)
		{
			Manager = manager;
			UIElement = CreateUIElement();
		}

		public UIElement UIElement { get; private set; }

		public EditableAlteringManager Manager { get; private set; }

		public PointNotifyPropertyChanged Position { get; private set; }

		private bool myIsVisible;
		public bool IsVisible {
			get
			{
				return myIsVisible;
			}
			set
			{
				myIsVisible = value;
				UIElement.Visibility = myIsVisible ? Visibility.Visible : Visibility.Hidden;
			}
		}

		internal const int EditableAlteringObjectZOrder = 1000;

		static internal readonly Size ObjectSize = new Size(6, 6);

		virtual protected FrameworkElement NewAlteringFrameworkElement();

		private UIElement CreateUIElement()
		{
			FrameworkElement handle = NewAlteringFrameworkElement();
			handle.SetValue(Canvas.ZIndexProperty, EditableAlteringObjectZOrder);

			handle.SetValue(Canvas.WidthProperty, ObjectSize.Width);
			handle.SetValue(Canvas.HeightProperty, ObjectSize.Height);

			Binding xBinding = new Binding(PointNotifyPropertyChanged.XProperty);
			xBinding.Source = Position;
			xBinding.Converter = new EditableAlteringObjectUIElementPositionConverter();
			handle.SetBinding(Canvas.LeftProperty, xBinding);

			Binding yBinding = new Binding(PointNotifyPropertyChanged.YProperty);
			yBinding.Source = Position;
			yBinding.Converter = new EditableAlteringObjectUIElementPositionConverter();
			handle.SetBinding(Canvas.TopProperty, yBinding);

			return handle;
		}

	}

	[ValueConversion(typeof(double), typeof(double))]
	class EditableAlteringObjectUIElementPositionConverter : IValueConverter
	{

		public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value - EditableAlteringObject.ObjectSize.Width * .5;
		}

		public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value + EditableAlteringObject.ObjectSize.Width * .5;
		}
	}
}
