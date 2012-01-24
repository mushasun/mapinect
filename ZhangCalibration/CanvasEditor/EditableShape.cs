using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Shapes;

namespace CanvasEditor
{
	class EditableShape : EditableFrameworkElement
	{

		private Shape Shape
		{
			get
			{
				return (Shape)UIElement;
			}
		}

	}
}
