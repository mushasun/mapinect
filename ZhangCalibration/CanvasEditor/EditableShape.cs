using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Shapes;
using System.Windows;

namespace CanvasEditor
{
	public class EditableShape : EditableFrameworkElement
	{

		internal EditableShape(CanvasEditor editor, FrameworkElement element)
			: base(editor, element)
		{

		}

		private Shape Shape
		{
			get
			{
				return (Shape)UIElement;
			}
		}

	}
}
