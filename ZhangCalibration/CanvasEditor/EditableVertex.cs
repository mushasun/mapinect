using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Shapes;
using PropertyChangedLibrary;
using System;

namespace CanvasEditor
{
	class EditableVertex : EditableAlteringObject
	{

		public EditableVertex(EditableVertexManager manager)
		{
			Position = manager.EditableHandlePositionToPoint(handlePosition);
		}

		protected override FrameworkElement NewAlteringFrameworkElement()
		{
			return new 
		}

		private UIElement CreateVertexUIElement()
		{
			Rectangle handle = new Ellipse();
			handle.SetValue(Canvas.ZIndexProperty, EditableHandleZOrder);

			handle.SetValue(Canvas.WidthProperty, HandleSize.Width);
			handle.SetValue(Canvas.HeightProperty, HandleSize.Height);

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

}
