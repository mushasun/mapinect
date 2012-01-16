using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CanvasEditor
{
	public class CanvasEditor
	{

		public CanvasEditor(System.Windows.Controls.Canvas canvas,
			IEnumerable<System.Windows.FrameworkElement> editableElements)
		{
			this.Canvas = canvas;
			this.EditableElements = editableElements;

			canvas.MouseMove += new System.Windows.Input.MouseEventHandler(canvas_MouseMove);
			foreach (System.Windows.FrameworkElement element in EditableElements)
			{
				element.PreviewMouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(element_PreviewMouseLeftButtonDown);
			}
		}

		void canvas_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
		{
			System.Windows.Point mousePosition = e.GetPosition(Canvas);
			if (EditingElement != null)
			{
				double left = (double)EditingElement.GetValue(System.Windows.Controls.Canvas.LeftProperty);
				double top = (double)EditingElement.GetValue(System.Windows.Controls.Canvas.TopProperty);
				System.Windows.Vector difference = System.Windows.Point.Subtract(mousePosition, LastMousePosition);
				EditingElement.SetValue(System.Windows.Controls.Canvas.LeftProperty, left + difference.X);
				EditingElement.SetValue(System.Windows.Controls.Canvas.TopProperty, top + difference.Y);
			}
			LastMousePosition = mousePosition;
		}

		void element_PreviewMouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (EditingElement != null && EditableElements.Contains(sender))
			{
				EditingElement = (System.Windows.FrameworkElement)sender;
			}
		}

		public System.Windows.Controls.Canvas Canvas { get; set; }

		public IEnumerable<System.Windows.FrameworkElement> EditableElements { get; set; }

		private System.Windows.FrameworkElement EditingElement { get; set; }

		private System.Windows.Point LastMousePosition { get; set; }

	}
}
