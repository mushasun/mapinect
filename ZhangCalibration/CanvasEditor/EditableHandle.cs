using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Shapes;
using PropertyChangedLibrary;
using System;

namespace CanvasEditor
{

	enum EditableHandlePosition
	{
		First = -1,
		TopLeft,
		TopCenter,
		TopRight,
		MidRight,
		BottomRight,
		BottomCenter,
		BottomLeft,
		MidLeft,
		Count
	}

	class EditableHandle
	{

		public EditableHandle(EditableHandleManager manager, EditableHandlePosition handlePosition)
		{
			Manager = manager;
			HandlePosition = handlePosition;
			Position = new PointNotifyPropertyChanged(manager.EditableHandlePositionToPoint(handlePosition));
			UIElement = CreateHandleUIElement();
		}

		public UIElement UIElement { get; private set; }

		public EditableHandleManager Manager { get; private set; }

		public EditableHandlePosition HandlePosition { get; private set; }

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

		internal const int EditableHandleZOrder = 1000;

		static internal readonly Size HandleSize = new Size(6, 6);

		private UIElement CreateHandleUIElement()
		{
			Rectangle handle = new Rectangle();
			handle.SetValue(Canvas.ZIndexProperty, EditableHandleZOrder);

			handle.SetValue(Canvas.WidthProperty, HandleSize.Width);
			handle.SetValue(Canvas.HeightProperty, HandleSize.Height);

			Binding xBinding = new Binding(PointNotifyPropertyChanged.XProperty);
			xBinding.Source = Position;
			xBinding.Converter = new EditableHandleUIElementPositionConverter();
			handle.SetBinding(Canvas.LeftProperty, xBinding);

			Binding yBinding = new Binding(PointNotifyPropertyChanged.YProperty);
			yBinding.Source = Position;
			yBinding.Converter = new EditableHandleUIElementPositionConverter();
			handle.SetBinding(Canvas.TopProperty, yBinding);

			return handle;
		}

	}

	[ValueConversion(typeof(double), typeof(double))]
	class EditableHandleUIElementPositionConverter : IValueConverter
	{

		public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value - EditableHandle.HandleSize.Width * .5;
		}

		public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
		{
			return (double)value + EditableHandle.HandleSize.Width * .5;
		}
	}
}
